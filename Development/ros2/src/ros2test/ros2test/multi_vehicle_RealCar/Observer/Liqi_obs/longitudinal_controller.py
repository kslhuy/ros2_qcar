
"""
Platoon Longitudinal Controller
================================
Implements the exact nonlinear vehicle dynamics and LCC-based control law
from the IFAC paper (Section 5).

Reference
---------
Q. Li, S. Meng, F. Meng, C. Delattre, A. Zemouche,
"Distributed High-Gain Observer Design of Interconnected Nonlinear
 Systems for Vehicle Platoon Application", IFAC 2026.

Vehicle model (Rajamani 2006) — per vehicle i
---------------------------------------------
    dp_i / dt  = v_i
    dv_i / dt  = - (C_wi / m_i) * v_i^2  -  mu * g  +  T_i / (m_i * R_di)
    dT_i / dt  = (1 / tau_i) * [ -T_i  +  m_i * R_di * F_i ]

Control force F_i
-----------------
    Leader  (i = 1):  F_1 = alpha_0 * (v_h - v_1)
    Follower (i > 1): F_i = alpha_i * ( V(s_i) - v_i ) + beta_i * s_dot_i

    where  s_i     = p_{i-1} - p_i          (measurable gap)
           s_dot_i = v_{i-1} - v_i          (gap rate)
           V(s)    = piecewise-cosine spacing policy

Triangular-form state for the observer
--------------------------------------
    Leader  :  x_1 = [p_1, v_1, a_1]
    Follower:  x_i = [s_i, v_{si}, a_{si}]

    with a_i = dv_i / dt  (acceleration) and a_dot_i = da_i / dt (jerk).

    The nonlinear coupling for the distributed observer is:
        phi_i = a_dot_{i-1} - a_dot_i

    a_dot_i is provided by the vehicle dynamics as da_dt.
"""

import math
from typing import Optional, Dict, Any, Tuple

import numpy as np

from ...Controller.longitudinal_controllers import LongitudinalControllerBase
from ...Controller.lateral_controllers import LateralControllerBase


# ===========================================================================
# Default parameters — Table 1 of the paper  (4-HDV platoon)
# ===========================================================================

DEFAULT_MASS   = [2417.0, 2720.3, 2000.1, 2302.3]   # m_i   [kg]
DEFAULT_CW     = [0.6512, 0.6512, 0.6512, 0.6512]   # C_wi  [N·s^2/m^2]
DEFAULT_RD     = [0.3,    0.4,    0.3,    0.4   ]   # R_di  [m]
DEFAULT_TAU    = [0.30,   0.33,   0.36,   0.40  ]   # tau_i [s]
DEFAULT_ALPHA  = [0.5500, 0.5576, 0.5511, 0.5624]   # alpha_i
DEFAULT_BETA   = [0.0,    1.1815, 1.1876, 1.2047]   # beta_i   (leader=0)
DEFAULT_MU     = 0.015                                # rolling resistance
DEFAULT_G      = 9.81                                 # gravity [m/s^2]
DEFAULT_V_MAX  = 20.0                                 # free-flow speed [m/s]
DEFAULT_S_ST   = 5.0                                  # standstill gap [m]
DEFAULT_S_GO   = 35.0                                 # free-flow gap   [m]


# ===========================================================================
# Spacing policy  V(s)   (piecewise-cosine)
# ===========================================================================

def spacing_policy(s: float,
                   s_st: float = DEFAULT_S_ST,
                   s_go: float = DEFAULT_S_GO,
                   v_max: float = DEFAULT_V_MAX) -> float:
    """Piecewise-cosine spacing policy V(s) from the paper Eq. (28)."""
    if s <= s_st:
        return 0.0
    if s >= s_go:
        return v_max
    return 0.5 * v_max * (1.0 - math.cos(math.pi * (s - s_st) / (s_go - s_st)))


# ===========================================================================
# Nonlinear vehicle dynamics  (pure physics, no control law)
# ===========================================================================

class VehicleDynamics:
    """
    Standalone nonlinear longitudinal vehicle model (Rajamani 2006).

    State:   (p, v, T)   — position, velocity, torque
    Output:  (v, a, a_dot)  — velocity, acceleration, jerk

    Usage
    -----
    >>> dyn = VehicleDynamics(idx=1)          # vehicle 2 from paper table
    >>> dyn.reset(p=20.0, v=10.0, T=0.0)
    >>> v, a, a_dot = dyn.step(F_control, dt=0.01)
    """

    def __init__(self,
                 idx: int = 0,
                 mass: Optional[float] = None,
                 C_w: Optional[float] = None,
                 R_d: Optional[float] = None,
                 tau: Optional[float] = None,
                 mu: float = DEFAULT_MU,
                 g: float = DEFAULT_G):
        """
        Parameters
        ----------
        idx : int
            0-based vehicle index (0=leader). Used to pick defaults.
        mass, C_w, R_d, tau : float, optional
            Override the corresponding default from the paper's table.
        """
        i = max(0, min(int(idx), len(DEFAULT_MASS) - 1))
        self.mass = DEFAULT_MASS[i] if mass is None else float(mass)
        self.C_w  = DEFAULT_CW[i]   if C_w  is None else float(C_w)
        self.R_d  = DEFAULT_RD[i]   if R_d  is None else float(R_d)
        self.tau  = DEFAULT_TAU[i]  if tau  is None else float(tau)
        self.mu   = float(mu)
        self.g    = float(g)

        # Internal state
        self.p: float = 0.0
        self.v: float = 0.0
        self.T: float = 0.0

        # Derived outputs — cached from last step
        self.a: float = 0.0         # dv / dt
        self.a_dot: float = 0.0     # d^2v / dt^2  (jerk)

    # -- State getters / setters --------------------------------------------

    def reset(self, p: float = 0.0, v: float = 0.0, T: float = 0.0):
        self.p = float(p)
        self.v = float(v)
        self.T = float(T)
        self.a = 0.0
        self.a_dot = 0.0

    @property
    def state(self) -> Tuple[float, float, float]:
        return (self.p, self.v, self.T)

    # -- Right-hand side ----------------------------------------------------

    def _rhs(self, v: float, T: float, F: float) -> Tuple[float, float]:
        """
        Compute (dv_dt, dT_dt) for a given (v, T, F).

        dv_dt = -C_w/m * v^2  -  mu*g  +  T / (m * R_d)
        dT_dt = (1/tau) * [ -T  +  m * R_d * F ]
        """
        m   = self.mass
        Rd  = self.R_d
        dv  = (-self.C_w / m) * v * v  -  self.mu * self.g  +  T / (m * Rd)
        dT  = (1.0 / self.tau) * (-T  +  m * Rd * F)
        return dv, dT

    # -- RK4 integration ----------------------------------------------------

    def step(self, F: float, dt: float) -> Tuple[float, float, float]:
        """
        Advance the dynamics by *dt* seconds with control force *F*.

        Returns  (v, a, a_dot)  after the step.
        """
        dt = max(float(dt), 1e-9)

        # --- RK4 on (v, T) ---
        v0, T0 = self.v, self.T

        k1v, k1T = self._rhs(v0,               T0,               F)
        k2v, k2T = self._rhs(v0 + 0.5*dt*k1v, T0 + 0.5*dt*k1T, F)
        k3v, k3T = self._rhs(v0 + 0.5*dt*k2v, T0 + 0.5*dt*k2T, F)
        k4v, k4T = self._rhs(v0 +     dt*k3v, T0 +     dt*k3T, F)

        v1 = v0 + (dt / 6.0) * (k1v + 2.0*k2v + 2.0*k3v + k4v)
        T1 = T0 + (dt / 6.0) * (k1T + 2.0*k2T + 2.0*k3T + k4T)

        # --- position (Euler, exact because p_dot = v) ---
        p1 = self.p + dt * v0

        # --- acceleration and jerk ---
        a_old = self.a
        a_new = (v1 - v0) / dt                               # a = dv/dt
        a_dot_new = (a_new - a_old) / dt if dt > 1e-12 else 0.0   # da/dt

        # --- update internal state ---
        self.p = p1
        self.v = v1
        self.T = T1
        self.a = a_new
        self.a_dot = a_dot_new

        return v1, a_new, a_dot_new


# ===========================================================================
# Platoon Longitudinal Controller
# ===========================================================================

class PlatoonLongitudinalController(LongitudinalControllerBase):
    """
    Longitudinal controller implementing the paper's LCC platoon strategy.

    Wraps a VehicleDynamics instance and computes the control force F at every
    step using the paper's leader / follower laws, then returns a throttle
    command.

    Also exposes  phi_i = a_dot_{i-1} - a_dot_i  for the distributed
    high-gain observer to consume.

    Interface (LongitudinalControllerBase)
    ---------------------------------------
        compute_throttle(follower_state, leader_state, dt) -> float

    Observer coupling
    -----------------
        controller.phi          # last computed nonlinear function value
        controller.a_dot        # own jerk (da/dt)
        controller.leader_a_dot # leader's jerk (needed for phi when follower)
        controller.dynamics     # the underlying VehicleDynamics
    """

    def __init__(self,
                 vehicle_index: int = 0,
                 is_leader: bool = False,
                 # --- override any physical parameter ---
                 mass: Optional[float] = None,
                 C_w: Optional[float] = None,
                 R_d: Optional[float] = None,
                 tau: Optional[float] = None,
                 mu: float = DEFAULT_MU,
                 g: float = DEFAULT_G,
                 # --- control parameters ---
                 alpha: Optional[float] = None,
                 beta: Optional[float] = None,
                 v_h: float = 0.0,
                 v_max: float = DEFAULT_V_MAX,
                 s_st: float = DEFAULT_S_ST,
                 s_go: float = DEFAULT_S_GO,
                 # --- output mapping ---
                 max_throttle: float = 0.3,
                 torque_to_throttle: float = 0.02,
                 # --- bookkeeping ---
                 config: Optional[Dict[str, Any]] = None,
                 logger: Any = None):
        """
        Parameters
        ----------
        vehicle_index : int
            0-based (0 = leader in the paper). Picks physical defaults.
        is_leader : bool
            Leader uses F = alpha * (v_h - v); follower uses gap-based law.
        v_h : float
            Virtual-leader reference speed [m/s] (leader only).
        max_throttle, torque_to_throttle : float
            Map internal torque to throttle command.
        """
        self.logger = logger
        self.is_leader = bool(is_leader)

        # --- vehicle physics ---
        self.dynamics = VehicleDynamics(
            idx=vehicle_index, mass=mass, C_w=C_w, R_d=R_d, tau=tau, mu=mu, g=g,
        )

        # --- control gains ---
        i = max(0, min(int(vehicle_index), len(DEFAULT_ALPHA) - 1))
        self.alpha = DEFAULT_ALPHA[i] if alpha is None else float(alpha)
        self.beta  = DEFAULT_BETA[i]  if beta  is None else float(beta)
        self.v_h   = float(v_h)
        self.v_max = float(v_max)
        self.s_st  = float(s_st)
        self.s_go  = float(s_go)

        # --- output ---
        self.max_throttle       = float(max_throttle)
        self.torque_to_throttle = float(torque_to_throttle)
        self.prev_throttle      = 0.0

        # --- observer coupling (populated each step) ---
        self.a_dot: float = 0.0          # own jerk
        self.leader_a_dot: float = 0.0   # leader jerk (set externally by observer)
        self.phi: float = 0.0            # a_dot_{leader} - a_dot_self

    # ------------------------------------------------------------------
    # Control force  F_i
    # ------------------------------------------------------------------

    def compute_control_force(self,
                              s: Optional[float],
                              s_dot: Optional[float],
                              v: float) -> float:
        """
        Leader   :  F = alpha * (v_h - v)
        Follower :  F = alpha * (V(s) - v)  +  beta * s_dot
        """
        if self.is_leader:
            return self.alpha * (self.v_h - v)

        if s is None or s_dot is None:
            return 0.0

        V_s = spacing_policy(float(s), self.s_st, self.s_go, self.v_max)
        return self.alpha * (V_s - v) + self.beta * float(s_dot)

    # ------------------------------------------------------------------
    # LongitudinalControllerBase  interface
    # ------------------------------------------------------------------

    def compute_throttle(self,
                         follower_state: Dict[str, float],
                         leader_state: Optional[Dict[str, float]],
                         dt: float) -> float:
        """
        One step of the platoon controller.

        follower_state  must contain  'velocity' (and optionally 'x').
        leader_state    must contain  'x', 'velocity'  when not leader.

        Returns  throttle ∈ [-max_throttle, +max_throttle].
        """
        dt = max(float(dt), 1e-9)
        v = float(follower_state.get("velocity", self.dynamics.v))

        # --- spacing and gap rate (follower only) ---
        s: Optional[float] = None
        s_dot: Optional[float] = None

        if not self.is_leader and leader_state is not None:
            lead_x = float(leader_state.get("x", 0.0))
            lead_v = float(leader_state.get("velocity", 0.0))
            s      = lead_x - self.dynamics.p
            s_dot  = lead_v - v

        # --- control force ---
        F = self.compute_control_force(s, s_dot, v)

        # --- integrate vehicle dynamics ---
        v_new, a_new, a_dot_new = self.dynamics.step(F, dt)
        self.a_dot = a_dot_new

        # --- nonlinear function for the distributed observer ---
        self.phi = self.leader_a_dot - self.a_dot

        # --- torque → throttle ---
        throttle = float(np.clip(
            self.dynamics.T * self.torque_to_throttle,
            -self.max_throttle,
            self.max_throttle,
        ))
        self.prev_throttle = throttle
        return throttle

    # ------------------------------------------------------------------
    # Parameter updates & reset
    # ------------------------------------------------------------------

    def update_params(self, params: Dict[str, Any]):
        for k, v in params.items():
            if hasattr(self, k):
                setattr(self, k, v)
            elif hasattr(self.dynamics, k):
                setattr(self.dynamics, k, v)
        if self.logger:
            self.logger.logger.info(
                f"[PlatoonLongitudinalController] params updated: {params}"
            )

    def reset(self):
        self.dynamics.reset()
        self.prev_throttle = 0.0
        self.a_dot = 0.0
        self.leader_a_dot = 0.0
        self.phi = 0.0

    # ------------------------------------------------------------------
    # State accessors  (for the observer)
    # ------------------------------------------------------------------

    @property
    def position(self) -> float:
        return self.dynamics.p

    @property
    def velocity(self) -> float:
        return self.dynamics.v

    @property
    def torque(self) -> float:
        return self.dynamics.T

    @property
    def acceleration(self) -> float:
        return self.dynamics.a

    def get_state(self) -> Dict[str, float]:
        return {
            "position":     self.dynamics.p,
            "velocity":     self.dynamics.v,
            "torque":       self.dynamics.T,
            "acceleration": self.dynamics.a,
        }

    def set_state(self,
                  position: float = 0.0,
                  velocity: float = 0.0,
                  torque: float = 0.0):
        self.dynamics.reset(p=position, v=velocity, T=torque)


# ===========================================================================
# Dummy lateral controller  (placeholder for multi-vehicle simulations)
# ===========================================================================

class DummyLateralController(LateralControllerBase):
    """Returns zero steering — placeholder for pure longitudinal studies."""

    def __init__(self, config=None, logger=None):
        self.logger = logger

    def compute_steering(self,
                         follower_state: Dict[str, float],
                         leader_state: Optional[Dict[str, float]],
                         dt: float) -> float:
        return 0.0

    def update_params(self, params: Dict[str, Any]):
        pass

    def reset(self):
        pass
