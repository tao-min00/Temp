#! /usr/bin/env python
# -*- coding: utf-8 -*-
from geometry_msgs.msg import Twist
import rospy
import math
import numpy as np
from mvccep_image_localization.msg import base_localization
import matplotlib.pyplot as plt

# 预瞄算法的一些超参数
k = 0.1  # control gain
L = 0.145  # [m] Wheel base of vehicle
target_speed = 0.3  # m/s
max_steer = math.radians(40.0)  # [rad] max steering angle 角度转为弧度

# 车辆状态
class State:
    def __init__(self, t=0.0, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.t = t
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
class trajectory(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
# 预瞄算法函数
def stanley_control(state, cx, cy, cyaw, pind):

    ind, efa = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind
    k1 = cyaw[ind]
    theta_e = pi_2_pi(cyaw[ind] - state.yaw)
    theta_d = math.atan2(k * efa, state.v)
    delta = theta_e + theta_d

    return delta, ind, efa

# 角度转换函数,保持在[-Π,Π]之间
def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

# 找到最近的预瞄点索引,以及最近点(后边把这个当作跟踪误差)
def calc_target_index(state, cx, cy):

    # calc frant axle position
    fx = state.x + L * math.cos(state.yaw)  # 前轴fx的距离？
    fy = state.y + L * math.sin(state.yaw)

    # search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    mind = min(d)   # 找到最近点
    ind = d.index(mind)
    angle1 = math.atan2(fy - cy[ind], fx - cx[ind])
    angle2 = state.yaw
    tyaw = pi_2_pi(math.atan2(fy - cy[ind], fx - cx[ind]) - state.yaw)   # 朝向最近轨迹点的车的航向角  状态点-期望点 坐标 正负代表了需要左转还是右转 
    if tyaw > 0.0:  # 大于0需要左转
        mind = - mind

    return ind, mind


class MultiVehiclesControl(object):
    def __init__(self, robot_ids):
        self.robot_ids = robot_ids
        self.car_number = len(self.robot_ids)
        self.states = dict.fromkeys(['0', '1', '2', '3', '4'], State())
        self.sub = rospy.Subscriber("base_localization", base_localization, self.localtionCallback, queue_size=10)
        # self.pub = [rospy.Publisher('/robot_{}/cmd_vel'.format(int(id)), Twist, queue_size=10) for id in self.robot_ids]
        self.pub = {}
        for id in robot_ids:
            self.pub[id] = rospy.Publisher('/robot_{}/cmd_vel'.format(int(id)), Twist, queue_size=10)
        
        
    def TestCircleTrajectory(self):
        # 圆轨迹
        x=np.linspace(-1,1,1000)
        y=np.sqrt(1-np.square(x))
        cx=np.append(x,-x)
        cy=np.append(y,-y)
        cyaw = []
        for i, e in enumerate(cy):
            cyaw.append(cx[i] / cy[i])
            
        cx.tolist()
        cy.tolist()
        pass
    
    def TestLinearTrajectory(self, start_x, end_x, y):
        
        # 直线轨迹

        cx = np.linspace(start_x, end_x, 2000, endpoint=False)
        cx = cx.tolist()
        cy = [y] * len(cx)
        cyaw = [0] * len(cx)
        
        # 定义
        traj = trajectory(cx, cy, cyaw)
        cx = traj.x
        cy = traj.y
        cyaw = traj.yaw
        
        return traj, cx, cy, cyaw

    def TestTwoLinearTrajectory(self, start_x, end_x, y, angle_): #20221114 TaoMin
        
        # 直线轨迹
        #yaw_angle = math.pi * 0.25 #45度
        mid_x = (start_x + end_x) / 2

        cx1 = np.linspace(start_x, mid_x, 1000, endpoint = False)        
        cx1 = cx1.tolist()
        cy1 = np.linspace(y - (mid_x - start_x) * math.tan(angle_), y, 1000, endpoint = False)
        cy1 = cy1.tolist()
        cyaw1 = [angle_] * len(cx1)

        cx2 = np.linspace(mid_x, end_x, 1000, endpoint = False)
        cx2 = cx2.tolist()
        cy2 = [y] * len(cx2)
        cyaw2 = [0] * len(cx2)

        cx = cx1 + cx2
        cy = cy1 + cy2
        cyaw = cyaw1 + cyaw2

        # cx = np.linspace(start_x, end_x, 2000, endpoint=False)
        # cx = cx.tolist()
        # cy = [y] * len(cx)
        # cyaw = [0] * len(cx)
        
        # 定义
        traj = trajectory(cx, cy, cyaw)
        cx = traj.x
        cy = traj.y
        cyaw = traj.yaw
        
        return traj, cx, cy, cyaw
    
    def localtionCallback(self, msg):
            # Stores all states.
            robot_0_x = msg.robot_0.pos.x
            robot_0_y = msg.robot_0.pos.y
            robot_0_angle = msg.robot_0.angle
            # robot_0_twist_linear = msg.robot_0.twist_linear.x
            robot_0_twist_linear = target_speed
            self.states['0'] = State(msg.header.stamp.secs,robot_0_x,robot_0_y,robot_0_angle,robot_0_twist_linear)
            
            robot_1_x = msg.robot_1.pos.x
            robot_1_y = msg.robot_1.pos.y
            robot_1_angle = msg.robot_1.angle
            # robot_1_twist_linear = msg.robot_1.twist_linear.x
            robot_1_twist_linear = target_speed
            self.states['1'] = State(msg.header.stamp.secs,robot_1_x,robot_1_y,robot_1_angle,robot_1_twist_linear)
            
            robot_2_x = msg.robot_2.pos.x
            robot_2_y = msg.robot_2.pos.y
            robot_2_angle = msg.robot_2.angle
            # robot_2_twist_linear = msg.robot_2.twist_linear.x
            robot_2_twist_linear = target_speed
            self.states['2'] = State(msg.header.stamp.secs,robot_2_x,robot_2_y,robot_2_angle,robot_2_twist_linear)
            
            robot_3_x = msg.robot_3.pos.x
            robot_3_y = msg.robot_3.pos.y
            robot_3_angle = msg.robot_3.angle
            # robot_3_twist_linear = msg.robot_3.twist_linear.x
            robot_3_twist_linear = target_speed
            
            self.states['3'] = State(msg.header.stamp.secs,robot_3_x,robot_3_y,robot_3_angle,robot_3_twist_linear)
            
            robot_4_x = msg.robot_4.pos.x
            robot_4_y = msg.robot_4.pos.y
            robot_4_angle = msg.robot_4.angle
            # robot_4_twist_linear = msg.robot_4.twist_linear.x
            robot_4_twist_linear = target_speed
            
            self.states['4'] = State(msg.header.stamp.secs,robot_4_x,robot_4_y,robot_4_angle,robot_4_twist_linear)
            
            for id in self.robot_ids:
                if id == '0':
                    # _, cx, cy, cyaw = self.TestLinearTrajectory(-1,1,0.52)
                    _, cx, cy, cyaw = self.TestTwoLinearTrajectory(-1,1,0.52)
                elif id == '1':
                    # _, cx, cy, cyaw = self.TestLinearTrajectory(-1,1,0.26)
                    _, cx, cy, cyaw = self.TestTwoLinearTrajectory(-1,1,0.26)
                elif id == '2':
                    # _, cx, cy, cyaw = self.TestLinearTrajectory(-1,1,0)
                    _, cx, cy, cyaw = self.TestTwoLinearTrajectory(-1,1,0)
                elif id == '3':
                    # _, cx, cy, cyaw = self.TestLinearTrajectory(-1,1,-0.26)
                    angle = math.pi / 4
                    _, cx, cy, cyaw = self.TestTwoLinearTrajectory(-1,1,0,angle)
                elif id == '4':
                    # _, cx, cy, cyaw = self.TestLinearTrajectory(-1,1,-0.52)
                    _, cx, cy, cyaw = self.TestTwoLinearTrajectory(-1,1,-0.52)
                    
                # Tracking controller
                lastIndex = len(cx) - 1
                target_ind, mind = calc_target_index(self.states[id], cx, cy)
                
                if lastIndex > target_ind:
                    di, target_ind, d_error = stanley_control(self.states[id], cx, cy, cyaw, target_ind)
                    # print(target_ind)
                    # print(di)

                    vel_msg = Twist()
                    vel_msg.linear.x = target_speed
                    vel_msg.angular.z = target_speed * math.tan(di) / L

                    self.pub[id].publish(vel_msg)
                    rospy.loginfo("Publish robot_{} velocity command[%0.2f m/s, %0.2f rad/s],d_error:%0.2f m".format(int(id)), vel_msg.linear.x, vel_msg.angular.z, d_error)

                else:
                    # 输出跟踪结束
                    rospy.loginfo("robot_{} arrive the end point.".format(int(id)))
            r.sleep()
 

if __name__ == '__main__':
    try:
        # ROS Init
        # 创建节点
        rospy.init_node('trajectory_tracking_control', anonymous=True)
        r = rospy.Rate(30) # 30hz
        
        # Get params
        robot_ids = rospy.get_param('~robot_id')
        robot_ids = str(robot_ids)
        robot_ids = robot_ids.split(',')
        
        
        mg = MultiVehiclesControl(robot_ids)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass