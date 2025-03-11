#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import tkinter as tk

# 初始化 ROS 节点
rospy.init_node('friction_publisher')

# 创建四个 ROS 发布者，分别控制四个轮子的摩擦力
pub_11 = rospy.Publisher('/wheel_friction_11', Float32, queue_size=10)
pub_12 = rospy.Publisher('/wheel_friction_12', Float32, queue_size=10)
pub_21 = rospy.Publisher('/wheel_friction_21', Float32, queue_size=10)
pub_22 = rospy.Publisher('/wheel_friction_22', Float32, queue_size=10)

# Tkinter GUI 界面
root = tk.Tk()
root.title("Wheel Friction Controller")

# 记录按钮状态（摩擦力是否降低）
friction_states = {
    "wheel_11": False,
    "wheel_12": False,
    "wheel_21": False,
    "wheel_22": False,
}

# 记录勾选框状态（是否同步）
sync_vars = {
    "wheel_11": tk.BooleanVar(value=False),
    "wheel_12": tk.BooleanVar(value=False),
    "wheel_21": tk.BooleanVar(value=False),
    "wheel_22": tk.BooleanVar(value=False),
}


# 获取轮子组别
def get_wheel_group(wheel):
    group = [wheel]
    if sync_vars[wheel].get():
        for key in friction_states:
            if key != wheel and sync_vars[key].get():
                group.append(key)
    return group


# 切换摩擦力

def toggle_friction(wheel):
    new_state = not friction_states[wheel]
    group = get_wheel_group(wheel)

    for w in group:
        friction_states[w] = new_state

    publish_friction()
    update_buttons()


# 更新按钮文本
def update_buttons():
    for wheel, button in buttons.items():
        state_text = "滑动" if friction_states[wheel] else "正常"
        button.config(text=f"{wheel.replace('_', ' ').title()} ({state_text})",
                      relief=tk.SUNKEN if friction_states[wheel] else tk.RAISED)


# 发布摩擦力数据
def publish_friction():
    pub_11.publish(0.001 if friction_states["wheel_11"] else 1.0)
    pub_12.publish(0.001 if friction_states["wheel_12"] else 1.0)
    pub_21.publish(0.001 if friction_states["wheel_21"] else 1.0)
    pub_22.publish(0.001 if friction_states["wheel_22"] else 1.0)

    rospy.loginfo(f"Published Friction - W11: {0.001 if friction_states['wheel_11'] else 1.0}  "
                  f"W12: {0.001 if friction_states['wheel_12'] else 1.0}  "
                  f"W21: {0.001 if friction_states['wheel_21'] else 1.0}  "
                  f"W22: {0.001 if friction_states['wheel_22'] else 1.0}")


# 创建按钮和勾选框
tk.Label(root, text="Wheel Friction Control").pack()
buttons = {}
for wheel in friction_states.keys():
    frame = tk.Frame(root)
    frame.pack()

    buttons[wheel] = tk.Button(frame, text=f"{wheel.replace('_', ' ').title()} (正常)",
                               command=lambda w=wheel: toggle_friction(w))
    buttons[wheel].pack(side=tk.LEFT)

    tk.Checkbutton(frame, text="同步", variable=sync_vars[wheel]).pack(side=tk.RIGHT)

# 运行 GUI
root.mainloop()
