import time
from hal.content.qcar_functions import QCarEKF
from hal.products.mats import SDCSRoadMap
from pal.products.qcar import QCarGPS
from ros2test.controller import SpeedController, SteeringController
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from qcar2_interfaces.msg import MotorCommands
import numpy as np

class VehicleControl(Node):
    def __init__(self):
        super().__init__('vehicle_control')
        self.v_ref = 0.9
        self.K_p = 0.1
        self.K_i = 1
        self.K_stanley = 1
        self.nodeSequence = [10, 4, 20, 10]
        
        self.ENCODER_COUNTS_PER_REV = 720.0 # counts per revolution
        self.WHEEL_RADIUS = 0.033 # front/rear wheel radius in m
        self.PIN_TO_SPUR_RATIO = 0.09536679536679536
        self.CPS_TO_MPS = (1/(self.ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
            * self.PIN_TO_SPUR_RATIO * 2*np.pi * self.WHEEL_RADIUS)
        
        self.roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        initialPose = self.roadmap.get_node_pose(self.nodeSequence[0]).squeeze()
        
        self.gps = QCarGPS(initialPose=initialPose,calibrate=False)
        self.ekf = QCarEKF(x_0=initialPose)
        
        self.sub = self.create_subscription(Imu, '/qcar2_imu', self.imu_callback, 10)
        self.sub2 = self.create_subscription(JointState, '/qcar2_joint', self.joint_callback, 10)

        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        self.steeringController = SteeringController(
            waypoints=self.waypointSequence,
            k=self.K_stanley
        )
        
        self.t0 = time.time()
        self.t = 0
        self.motorTach = 0
        self.delta = 0 
        
        
    def joint_callback(self, msg): 
        # self.get_logger().info("joint " + str(msg.velocity[0] * self.CPS_TO_MPS))
        self.motorTach = msg.velocity[0] * self.CPS_TO_MPS
        
    def imu_callback(self, msg): 
        self.imu_data = msg.angular_velocity.z
        tp = self.t
        self.t = time.time() - self.t0
        dt = self.t-tp
        self.get_logger().info("dt " + str(dt))
        #endregion

        #region : Read from sensors and update state estimates
        if self.imu_data:
            if self.gps.readGPS():
                y_gps = np.array([
                    self.gps.position[0],
                    self.gps.position[1],
                    self.gps.orientation[2]
                ])
                self.ekf.update(
                    [self.motorTach, self.delta],
                    dt,
                    y_gps,
                    self.imu_data,
                )
                
            else:
                self.ekf.update(
                    [self.motorTach, self.delta],
                    dt,
                    None,
                    self.imu_data,
                )

            x = self.ekf.x_hat[0,0]
            y = self.ekf.x_hat[1,0]
            
            th = self.ekf.x_hat[2,0]
            p = ( np.array([x, y])
                + np.array([np.cos(th), np.sin(th)]) * 0.2)
        v = self.motorTach
        #endregion

        u = self.speedController.update(v, self.v_ref, dt)
        self.delta = self.steeringController.update(p, th, v)

        self.send_control(u, self.delta)

    def send_control(self, motor, steer):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(steer)
        msg.values.append(motor)
        self.pub.publish(msg)
        self.get_logger().info("motor " + str(motor))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
