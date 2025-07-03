import time
from hal.products.mats import SDCSRoadMap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from qcar2_interfaces.msg import MotorCommands
import numpy as np
from geometry_msgs.msg import PoseStamped
from hal.utilities.control import StanleyController
from ros2test.controller import SpeedController
from ros2test.util import quaternion_to_yaw

class VehicleControl(Node):
    def __init__(self):
        super().__init__('vehicle_control')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_ref', 0.9),
                ('K_p', 0.1),
                ('K_i', 1),
                ('K_stanley', 1),
                ('NODE_SEQUENCE', [10, 4, 20, 10]),
                ('ENCODER_COUNTS_PER_REV', 720.0),
                ('WHEEL_RADIUS', 0.033),
                ('PIN_TO_SPUR_RATIO', 0.09536679536679536),
            ]
        )        
        
        # Get parameters
        self.v_ref = self.get_parameter("v_ref").value
        self.K_p = self.get_parameter("K_p").value
        self.K_i = self.get_parameter("K_i").value
        self.K_stanley = self.get_parameter("K_stanley").value
        self.nodeSequence = self.get_parameter("NODE_SEQUENCE").value
        self.ENCODER_COUNTS_PER_REV = self.get_parameter("ENCODER_COUNTS_PER_REV").value
        self.WHEEL_RADIUS = self.get_parameter("WHEEL_RADIUS").value
        self.PIN_TO_SPUR_RATIO = self.get_parameter("PIN_TO_SPUR_RATIO").value
        
        self.CPS_TO_MPS = (1/(self.ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
            * self.PIN_TO_SPUR_RATIO * 2*np.pi * self.WHEEL_RADIUS)
        
        roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = roadmap.generate_path(self.nodeSequence)
        
        self.sub = self.create_subscription(PoseStamped, '/ekf_pose', self.ekf_callback, 10)
        self.sub2 = self.create_subscription(JointState, '/qcar2_joint', self.joint_callback, 10)

        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        
        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        self.stanleyController = StanleyController(
            waypoints=self.waypointSequence,
            k=self.K_stanley
        )
        
        self.t0 = time.time()
        self.t = 0
        self.motorTach = 0
        self.delta = 0 
        
        
    def joint_callback(self, msg: JointState): 
        # self.get_logger().info("joint " + str(msg.velocity[0] * self.CPS_TO_MPS))
        self.motorTach = msg.velocity[0] * self.CPS_TO_MPS
        
    def ekf_callback(self, msg: PoseStamped): 
        pose = msg.pose
        tp = self.t
        self.t = time.time() - self.t0
        dt = self.t-tp

        x = pose.position.x
        y = pose.position.y
        
        ox = pose.orientation.x
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        
        th = quaternion_to_yaw(ox, oy, oz, ow)
        
        p = ( np.array([x, y])
            + np.array([np.cos(th), np.sin(th)]) * 0.2)
        v = self.motorTach

        u = self.speedController.update(v, self.v_ref, dt)
        self.delta = self.stanleyController.update(p, th, v)

        self.send_control(u, self.delta)

    def send_control(self, motor, steer):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(steer)
        msg.values.append(motor)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
