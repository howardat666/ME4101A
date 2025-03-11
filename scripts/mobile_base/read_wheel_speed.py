import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 初始化存储数据的列表
positions = [[], [], [], []]  # 4个车轮的位置
velocities = [[], [], [], []]  # 4个车轮的速度
time_stamps = []  # 时间戳，用于X轴


# 定义回调函数，处理订阅到的消息
def joint_state_callback(msg):
    # 提取出位置和速度
    if len(msg.position) == 6 and len(msg.velocity) == 6:
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        time_stamps.append(current_time)

        for i in range(4):
            positions[i].append(msg.position[i])
            velocities[i].append(msg.velocity[i])


# 初始化ROS节点
rospy.init_node('wheel_state_listener')

# 订阅/robot/joint_states话题
rospy.Subscriber("/robot/joint_states", JointState, joint_state_callback)

# 设置Matplotlib实时绘图
fig, ax = plt.subplots(2, 1, figsize=(10, 8))  # 创建2个子图，分别用于位置和速度


# 实时更新函数，用于更新绘图
def update_plot(frame):
    ax[0].clear()
    ax[1].clear()

    if len(time_stamps) > 0:
        # 绘制位置数据
        for i in range(4):
            min_length = min(len(time_stamps), len(positions[i]))
            ax[0].plot(time_stamps[:min_length], positions[i][:min_length], label=f'Link {(i + 2)//2}{2-(i + 1)%2} Position')
        ax[0].set_title('Wheel Positions')
        ax[0].set_ylabel('Position (rad)')
        ax[0].legend()

        # 绘制速度数据
        for i in range(4):
            min_length = min(len(time_stamps), len(velocities[i]))
            ax[1].plot(time_stamps[:min_length], velocities[i][:min_length], label=f'Wheel {i + 1} Velocity')
        ax[1].set_title('Wheel Velocities')
        ax[1].set_ylabel('Velocity (rad/s)')
        ax[1].legend()



# 使用FuncAnimation实现实时绘图，间隔100ms更新一次
ani = FuncAnimation(fig, update_plot, interval=100)

# 显示绘图
plt.show()

# 保持ROS运行
rospy.spin()
