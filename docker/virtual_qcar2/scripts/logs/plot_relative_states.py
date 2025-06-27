import numpy as np
import matplotlib.pyplot as plt

# Load saved data
# data = np.load("/home/qcar2_scripts/logs/relative_states.npz")
data = np.load("relative_states.npz")

t = data["time"]
pos_l = data["pos_leader"]
pos_f = data["pos_follower"]
vel_l = data["vel_leader"]
vel_f = data["vel_follower"]
acc_l = data["acc_leader"]
acc_f = data["acc_follower"]
spacing_th = data["spacing_theory"]

# Compute relative values (X axis only)
rel_pos = pos_l[:,0] - pos_f[:,0]
rel_vel = vel_l[:,0] - vel_f[:,0]
rel_acc = acc_l[:,0] - acc_f[:,0]

# Plot
plt.figure(figsize=(10, 8))

# plt.subplot(3,1,1)
# plt.plot(t, rel_pos)
# plt.ylabel("m")
# plt.title("Relative Position X history")
# plt.grid(True)

plt.subplot(3, 1, 1)
plt.plot(t, rel_pos, label="Actual Δ")
plt.plot(t, spacing_th, label="Theoretical Δ", linestyle="--")
plt.ylabel("Distance [m]")
plt.title("Relative Position vs Theoretical Spacing")
plt.grid(True)
plt.legend()


plt.subplot(3,1,2)
plt.plot(t, rel_vel)
plt.ylabel("m/s")
plt.title("Relative Speed history")
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(t, rel_acc)
plt.ylabel("m/s²")
plt.xlabel("Time (s)")
plt.title("Relative Accel history")
plt.grid(True)

plt.tight_layout()
plt.show()
