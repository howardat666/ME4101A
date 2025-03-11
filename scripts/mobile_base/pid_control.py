import rospy
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import threading
from tf.transformations import euler_from_quaternion
from simple_pid import PID

# 初始化全局变量
theta1 = theta2 = 0
theta = 0  # 由IMU的yaw值来更新
actual_Vx = actual_Vy = actual_w = 0

# 目标小车输入参数
Vx = float(input("请输入 Vx 的值: "))
Vy = float(input("请输入 Vy 的值: "))
w = float(input("请输入 w 的值: "))

# 轮子参数
l1 = 0.465
l2 = 0.075

# PID控制器初始化 (加入前馈)
pid_Vx = PID(1, 0.1, 0.3)
pid_Vy = PID(1, 0.1, 0.3)
pid_w = PID(1.2, 0.05, 0.2)

# 设定目标值 (前馈项)
pid_Vx.setpoint = 0  # 目标误差为 0
pid_Vy.setpoint = 0
pid_w.setpoint = 0

# 角度回调函数
def angle_callback(data):
    global theta1, theta2
    joint_index1 = data.name.index("wheel_swirl_joint_2")
    theta1 = -data.position[joint_index1]
    joint_index2 = data.name.index("wheel_swirl_joint_1")
    theta2 = data.position[joint_index2]

# IMU回调函数
def imu_callback(data):
    global theta, actual_w
    quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    _, _, yaw = euler_from_quaternion(quaternion)
    theta = -yaw
    actual_w = -data.angular_velocity.z

# 里程计回调函数
def odom_callback(data):
    global actual_Vx, actual_Vy
    actual_Vx = data.twist.twist.linear.x / 0.105
    actual_Vy = data.twist.twist.linear.y / 0.105

# 计算轮子速度的函数
def compute_wheel_speeds(Vx, Vy, w):
    Vx1 = Vx + w * l1 * np.cos(theta)
    Vy1 = Vy - w * l1 * np.sin(theta)
    Vx2 = Vx - w * l1 * np.cos(-theta)
    Vy2 = Vy - w * l1 * np.sin(-theta)

    t1 = theta1 + theta
    t2 = theta2 - theta

    VA = Vx1 * np.sin(t1) + Vy1 * np.cos(t1) + Vx1 * np.cos(t1) - Vy1 * np.sin(t1)
    VB = Vx1 * np.sin(t1) + Vy1 * np.cos(t1) - Vx1 * np.cos(t1) + Vy1 * np.sin(t1)
    VC = Vx2 * np.sin(t2) - Vy2 * np.cos(t2) + Vx2 * np.cos(t2) + Vy2 * np.sin(t2)
    VD = Vx2 * np.sin(t2) - Vy2 * np.cos(t2) - Vx2 * np.cos(t2) - Vy2 * np.sin(t2)

    return VA, VB, VC, VD

# 发布轮子速度
def publish_wheel_speeds(rate_hz=100, duration=600):
    pub_left_front = rospy.Publisher('/robot/left_front_wheel_controller/command', Float64, queue_size=10)
    pub_right_front = rospy.Publisher('/robot/right_front_wheel_controller/command', Float64, queue_size=10)
    pub_left_rear = rospy.Publisher('/robot/left_rear_wheel_controller/command', Float64, queue_size=10)
    pub_right_rear = rospy.Publisher('/robot/right_rear_wheel_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        if current_time - start_time >= duration:
            msg_zero = Float64(data=0.0)
            pub_left_front.publish(msg_zero)
            pub_right_front.publish(msg_zero)
            pub_left_rear.publish(msg_zero)
            pub_right_rear.publish(msg_zero)
            rospy.loginfo("All wheels set to 0. Shutting down.")
            break

        # 计算误差
        error_Vx =-Vx + actual_Vx
        error_Vy =-Vy + actual_Vy
        error_w =-w + actual_w

        # PID 控制修正（反馈项）
        correction_Vx = pid_Vx(error_Vx)
        correction_Vy = pid_Vy(error_Vy)
        correction_w = pid_w(error_w)

        # 计算总控制量 = 前馈 + 反馈
        corrected_Vx = Vx + correction_Vx
        corrected_Vy = Vy + correction_Vy
        corrected_w = w + correction_w

        # print(actual_Vy,actual_Vy)
        print(corrected_Vx,corrected_Vy,corrected_w)

        # 计算轮子速度
        VA, VB, VC, VD = compute_wheel_speeds(corrected_Vx, corrected_Vy, corrected_w)
        pub_left_front.publish(Float64(data=VB))
        pub_right_front.publish(Float64(data=VA))
        pub_left_rear.publish(Float64(data=VD))
        pub_right_rear.publish(Float64(data=VC))

        # rospy.loginfo(f"Published speeds: VA={VA:.2f}, VB={VB:.2f}, VC={VC:.2f}, VD={VD:.2f}")
        # rospy.loginfo(f"theta1: {theta1:.2f}, theta2: {theta2:.2f}, theta (yaw): {theta:.4f}")
        rate.sleep()

# 主函数
def main():
    rospy.init_node('wheel_speed_controller_with_pid_feedforward', anonymous=True)
    rospy.Subscriber("/robot/joint_states", JointState, angle_callback)
    rospy.Subscriber("/robot/imu_data", Imu, imu_callback)
    rospy.Subscriber("/odom_rf2o", Odometry, odom_callback)

    publish_thread = threading.Thread(target=publish_wheel_speeds)
    publish_thread.start()
    publish_thread.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
