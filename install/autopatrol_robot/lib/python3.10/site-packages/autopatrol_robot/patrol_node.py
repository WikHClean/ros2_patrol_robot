from geometry_msgs.msg import PoseStamped,Pose
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import rclpy.time
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion,quaternion_from_euler
import math


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        #声明相关参数
        self.declare_parameter('initial_point',[0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_pose_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        return PoseStamped对象
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        #返回顺序是 x,y,z,w ， 有的函数为 w,x,y,z
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose


    def init_robot_pose(self):
        """
        初始化机器人的位置
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(
            self.initial_point_[0],
            self.initial_point_[1],
            self.initial_point_[2]
        )
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()#等待导航系统激活
    
    def get_target_points(self):
        """
        通过参数值获取目标点的集合
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger.info(f"获取到目标点:{index}->{x},{y},{yaw}")
        return points


    
    def self_to_pose(self,target_point):
        """
        导航到指定的目标点
        """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f'剩余距离:{feedback.distance_remaining}')
        result = self.getResult()
        self.get_logger().info(f'导航结果：{result}')
    
    def get_current_pose(self):
        """
        获取当前机器人的位置
        """
        while rclpy.ok():
            
            try:
                result = self.buffer_.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = result.transform
                self.get_logger().info(f'平移:{transform.translation}')
                # rotation_euler = euler_from_quaternion([
                #     transform.rotation.x,
                #     transform.rotation.y,
                #     transform.rotation.z,
                #     transform.rotation.w
                # ])
                # self.get_logger().info(
                #     f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')
    




def main():
    rclpy.init()
    patrol = PatrolNode()
    # rclpy.spin(patrol)
    patrol.init_robot_pose()
    

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            x,y,yaw = point[0],point[1],point[2]
            target_pose = patrol.get_pose_by_xyyaw(x,y,yaw)
            patrol.nav_to_pose(target_pose)
    
    rclpy.shutdown()