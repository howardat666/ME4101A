import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
from scipy.signal import savgol_filter  # 导入 Savitzky-Golay 滤波器

# Initialize ROS node
rospy.init_node('odometry_plotter')

# Define global variables and thread lock
vx_data = []
vy_data = []
w_data = []  # 角速度数据
vx_filtered = []
vy_filtered = []
w_filtered = []
time_data = []
imu_time_data = []  # 用于保存IMU数据的时间戳
imu_data_buffer = []  # 用于存储IMU数据
buffer_size = 800  # 限制数据长度，防止内存占用过高

# Filter parameters for Savitzky-Golay filter
filter_window_length = 11  # 必须为奇数
filter_polyorder = 2       # 多项式阶数

data_lock = threading.Lock()  # 用于保证数据更新与读取的线程安全

# 创建发布器，将滤波后的数据发布到 "/calculated_velocity" 话题，类型为 Twist
velocity_pub = rospy.Publisher('/calculated_velocity', Twist, queue_size=10)

# 定义一个函数，使用 Savitzky-Golay 滤波器平滑数据
def apply_savgol_filter(data, window_length, polyorder):
    """
    对数据应用 Savitzky-Golay 滤波，若数据长度不足，则返回原始数据
    """
    if len(data) < window_length:
        return data
    return list(savgol_filter(np.array(data), window_length, polyorder))


# 里程计回调函数
def odometry_callback(msg):
    global vx_data, vy_data, vx_filtered, vy_filtered, time_data, imu_time_data, imu_data_buffer
    with data_lock:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        current_time = rospy.get_time()

        vx_data.append(vx)
        vy_data.append(vy)
        time_data.append(current_time)

        if len(time_data) > buffer_size:
            vx_data.pop(0)
            vy_data.pop(0)
            time_data.pop(0)

        # 对速度数据应用Savitzky-Golay滤波器
        vx_filtered[:] = apply_savgol_filter(vx_data, filter_window_length, filter_polyorder)
        vy_filtered[:] = apply_savgol_filter(vy_data, filter_window_length, filter_polyorder)

        # 查找与当前时间戳最接近的IMU数据
        if len(imu_data_buffer) > 0:
            closest_imu_data = find_closest_imu_data(current_time)
            w_data.append(closest_imu_data['w'])  # 将找到的角速度添加到w_data
            w_filtered[:] = apply_savgol_filter(w_data, filter_window_length, filter_polyorder)  # 对角速度进行滤波

            # 发布数据
            twist_msg = Twist()
            twist_msg.linear.x = vx_filtered[-1]
            twist_msg.linear.y = vy_filtered[-1]
            velocity_pub.publish(twist_msg)


# IMU回调函数
def imu_callback(msg):
    global imu_time_data, imu_data_buffer
    with data_lock:
        w = msg.angular_velocity.z  # 读取IMU的z轴角速度
        current_time = rospy.get_time()

        # 将IMU数据与时间戳保存
        imu_time_data.append(current_time)
        imu_data_buffer.append({'time': current_time, 'w': w})

        if len(imu_time_data) > buffer_size:
            imu_data_buffer.pop(0)
            imu_time_data.pop(0)


# 查找与目标时间戳最接近的IMU数据
def find_closest_imu_data(target_time):
    closest_time_idx = np.argmin(np.abs(np.array(imu_time_data) - target_time))
    return imu_data_buffer[closest_time_idx]


# 订阅话题
rospy.Subscriber('/odom_rf2o', Odometry, odometry_callback)
rospy.Subscriber('/robot/imu_data', Imu, imu_callback)

# 设置 Matplotlib 绘图
fig, (ax_vx, ax_vy, ax_w) = plt.subplots(3, 1, figsize=(10, 8))

def update_plot(frame):
    with data_lock:
        t = time_data.copy()
        vx_orig = vx_data.copy()
        vx_filt = vx_filtered.copy()
        vy_orig = vy_data.copy()
        vy_filt = vy_filtered.copy()
        w_orig = w_data.copy()
        w_filt = w_filtered.copy()

    min_length = min(len(t), len(vx_orig), len(vx_filt), len(vy_orig), len(vy_filt), len(w_orig), len(w_filt))
    if min_length == 0:
        return

    t = t[-min_length:]
    vx_orig = vx_orig[-min_length:]
    vx_filt = vx_filt[-min_length:]
    vy_orig = vy_orig[-min_length:]
    vy_filt = vy_filt[-min_length:]
    w_orig = w_orig[-min_length:]
    w_filt = w_filt[-min_length:]

    ax_vx.clear()
    ax_vy.clear()
    ax_w.clear()

    ax_vx.plot(t, vx_orig, label="VX Raw", color='lightblue', linestyle='dashed')
    ax_vx.plot(t, vx_filt, label="VX Filtered", color='blue')
    ax_vx.set_title('VX vs Time')
    ax_vx.set_xlabel('Time (s)')
    ax_vx.set_ylabel('VX (m/s)')
    ax_vx.legend(loc='upper left')
    ax_vx.grid(True)

    ax_vy.plot(t, vy_orig, label="VY Raw", color='lightgreen', linestyle='dashed')
    ax_vy.plot(t, vy_filt, label="VY Filtered", color='green')
    ax_vy.set_title('VY vs Time')
    ax_vy.set_xlabel('Time (s)')
    ax_vy.set_ylabel('VY (m/s)')
    ax_vy.legend(loc='upper left')
    ax_vy.grid(True)

    ax_w.plot(t, w_orig, label="W Raw", color='lightcoral', linestyle='dashed')
    ax_w.plot(t, w_filt, label="W Filtered", color='red')
    ax_w.set_title('W vs Time')
    ax_w.set_xlabel('Time (s)')
    ax_w.set_ylabel('W (rad/s)')
    ax_w.legend(loc='upper left')
    ax_w.grid(True)

ani = FuncAnimation(fig, update_plot, interval=100)
plt.ion()
plt.tight_layout()
plt.show()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    plt.pause(0.1)
    rate.sleep()
