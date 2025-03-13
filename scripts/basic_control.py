import rospy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
import numpy as np
import threading
from tf.transformations import euler_from_quaternion

# 初始化全局变量
theta1 = theta2 = 0
theta = 0  # 由IMU的yaw值来更新

# 回调函数：更新关节角度
def angle_callback(data):
    global theta1, theta2
    joint_index1 = data.name.index("wheel_swirl_joint_2")
    theta1 = -data.position[joint_index1]
    joint_index2 = data.name.index("wheel_swirl_joint_1")
    theta2 = data.position[joint_index2]

# IMU回调函数：更新theta
def imu_callback(data):
    global theta
    # 提取四元数并转换为欧拉角
    quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    _, _, yaw = euler_from_quaternion(quaternion)
    theta = -yaw  # 使用yaw作为theta

# 小车输入参数
Vx = float(input("请输入 Vx 的值: "))
Vy = float(input("请输入 Vy 的值: "))
w = float(input("请输入 w 的值: "))
l1 = 0.465
l2 = 0.075

# 计算轮子速度的函数
def compute_wheel_speeds():
    # 计算 Vx', Vy', Vx'', Vy''
    Vx1 = Vx + w * l1 * np.cos(theta)
    Vy1 = Vy - w * l1 * np.sin(theta)
    Vx2 = Vx - w * l1 * np.cos(-theta)
    Vy2 = Vy - w * l1 * np.sin(-theta)

    # 修正 theta1, theta2
    t1 = theta1 + theta
    t2 = theta2 - theta

    # 计算 VA, VB, VC, VD
    VA = Vx1 * np.sin(t1) + Vy1 * np.cos(t1) + Vx1 * np.cos(t1) - Vy1 * np.sin(t1)
    VB = Vx1 * np.sin(t1) + Vy1 * np.cos(t1) - Vx1 * np.cos(t1) + Vy1 * np.sin(t1)
    VC = Vx2 * np.sin(t2) - Vy2 * np.cos(t2) + Vx2 * np.cos(t2) + Vy2 * np.sin(t2)
    VD = Vx2 * np.sin(t2) - Vy2 * np.cos(t2) - Vx2 * np.cos(t2) - Vy2 * np.sin(t2)

    return VA, VB, VC, VD

# ROS发布函数，循环发布速度
def publish_wheel_speeds(rate_hz=100):
    pub_left_front = rospy.Publisher('/robot/left_front_wheel_controller/command', Float64, queue_size=10)
    pub_right_front = rospy.Publisher('/robot/right_front_wheel_controller/command', Float64, queue_size=10)
    pub_left_rear = rospy.Publisher('/robot/left_rear_wheel_controller/command', Float64, queue_size=10)
    pub_right_rear = rospy.Publisher('/robot/right_rear_wheel_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        # 计算并发布当前时刻的轮子速度
        VA, VB, VC, VD = compute_wheel_speeds()
        pub_left_front.publish(Float64(data=VB))
        pub_right_front.publish(Float64(data=VA))
        pub_left_rear.publish(Float64(data=VD))
        pub_right_rear.publish(Float64(data=VC))

        rospy.loginfo(f"Published speeds: VA={VA:.2f}, VB={VB:.2f}, VC={VC:.2f}, VD={VD:.2f}")
        rospy.loginfo(f"theta1: {theta1:.2f}, theta2: {theta2:.2f}, theta (yaw): {theta:.4f}")
        rate.sleep()

# 主函数
def main():
    rospy.init_node('wheel_speed_controller', anonymous=True)
    rospy.Subscriber("/robot/joint_states", JointState, angle_callback)
    rospy.Subscriber("/robot/imu_data", Imu, imu_callback)  # 订阅IMU数据更新theta

    # 启动发布线程
    publish_thread = threading.Thread(target=publish_wheel_speeds)
    publish_thread.start()

    publish_thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
