#!/usr/bin/env python

import rospy
from prius_msgs.msg import My_Trajectory
from prius_msgs.msg import Augmented_My_Trajectory_Point
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, atan2

DESIRED_TRAJECTORY_DATA_NUM = 10
POSE_INFO_NUM = 13
PI = 3.1415926


def ReadTrajectory(planned_trajectory):
    length = len(planned_trajectory.trajectory_points)
    trajectory_info = np.zeros((DESIRED_TRAJECTORY_DATA_NUM,length),dtype=float)
    for i in range(length):
        trajectory_info[0,i]=planned_trajectory.trajectory_points[i].x
        trajectory_info[1,i]=planned_trajectory.trajectory_points[i].y
        trajectory_info[2,i]=planned_trajectory.trajectory_points[i].z
        trajectory_info[3,i]=planned_trajectory.trajectory_points[i].theta
        trajectory_info[4,i]=planned_trajectory.trajectory_points[i].kappa
        trajectory_info[5,i]=planned_trajectory.trajectory_points[i].s
        trajectory_info[6,i]=planned_trajectory.trajectory_points[i].dkappa
        trajectory_info[7,i]=planned_trajectory.trajectory_points[i].v
        trajectory_info[8,i]=planned_trajectory.trajectory_points[i].a
        trajectory_info[9,i]=planned_trajectory.trajectory_points[i].relative_time
    return trajectory_info

def CreateRoad(planned_trajectory,road_width):
    length = len(planned_trajectory.trajectory_points)
    road= np.zeros((4,length),dtype=float)
    for i in range(length):
        point = planned_trajectory.trajectory_points[i]
        angle_right = point.theta-PI/2
        angle_left = angle_right-PI
        road[0,i]=point.x+road_width/2*cos(angle_left)
        road[1,i]=point.y+road_width/2*sin(angle_left)
        road[2,i]=point.x+road_width/2*cos(angle_right)
        road[3,i]=point.y+road_width/2*sin(angle_right)
    return road

def ReadPose(current_info):
    temp = np.zeros((POSE_INFO_NUM,1),dtype=float)
    temp[0]=current_info.trajectory_point.x
    temp[1]=current_info.trajectory_point.y
    temp[2]=current_info.trajectory_point.v
    temp[3]=current_info.trajectory_point.theta
    temp[4]=current_info.trajectory_point.kappa
    temp[5]=current_info.trajectory_point.relative_time
    temp[6]=current_info.goal_x
    temp[7]=current_info.goal_y
    temp[8]=current_info.preview_x
    temp[9]=current_info.preview_y
    temp[10]=current_info.distance_error
    temp[11]=current_info.heading_error
    temp[12]=current_info.velocity_error
    return temp


def TrajectoryPlotter():
    
    fig=plt.figure(1)
    
    planned_trajectory = rospy.wait_for_message("/prius/planning_output", My_Trajectory, timeout=None)
    trajectory_info = ReadTrajectory(planned_trajectory)

    controllers_num=rospy.get_param("/controllers_num")
    longitudinal_controller=rospy.get_param("/longitudinal_controller")  
    lateral_controller=rospy.get_param("/lateral_controller")
    centralized_controller=rospy.get_param("/centralized_controller")
    trajectory_velocity=rospy.get_param("/trajectory_velocity")
    
    if controllers_num==2:
        if trajectory_velocity=="record":
            label = longitudinal_controller+" and "+lateral_controller+"\n"+"curve velocity is not constant"
        else:
            label = longitudinal_controller+" and "+lateral_controller+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
    elif controllers_num==1:
        if trajectory_velocity=="record":
            label = centralized_controller+"\n"+"curve velocity is not constant"
        else:
            label = centralized_controller+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
    loc='center'
    font_dict={'fontsize': 20,\
            'fontweight' : 8.2,\
            'verticalalignment': 'baseline',\
            'horizontalalignment': loc}

    if trajectory_velocity=="record": 
        xmin=-300
        xmax=250
        ymin=-250
        ymax=300      
    else:
        xmin=-300
        xmax=250
        ymin=-20
        ymax=300

    zoom = (xmax-xmin)/200
    road_width=3.75*zoom   
    

    road = CreateRoad(planned_trajectory,road_width)

    pose_info = np.zeros((POSE_INFO_NUM,1),dtype=float)
    while not rospy.is_shutdown():
        current_info = rospy.wait_for_message("/prius/vehicle_info", Augmented_My_Trajectory_Point, timeout=None)

        
        temp = ReadPose(current_info)
        pose_info = np.hstack([pose_info,temp])

        
        plt.plot(trajectory_info[0,:],trajectory_info[1,:],"--",c="c",linewidth=2.0,label="planned trajectory")
        plt.plot(road[0,:],road[1,:],c="c",linewidth=3.0)
        plt.plot(road[2,:],road[3,:],c="c",linewidth=3.0)

        plt.plot(pose_info[0,1:-1],pose_info[1,1:-1],c="r",linewidth=2.0,label="actual trajectory")
        plt.plot(current_info.goal_x,current_info.goal_y,"bo",label = "goal point",)
        plt.plot(current_info.preview_x,current_info.preview_y,"go",label="preview point")
        plt.title(label,fontdict=font_dict,loc=loc)
        plt.text(-285, 290, "distance error = "+str(round(current_info.distance_error,4))+" m",fontsize=14) 
        plt.text(-285, 280, "heading error = "+str(round(current_info.heading_error*180/PI,4))+" degree",fontsize=14) 
        plt.text(-285, 270, "velocity error = "+str(round(current_info.velocity_error,4))+" m/s",fontsize=14) 
        plt.text(-285, 260, "current velocity = "+str(round(current_info.trajectory_point.v,4))+" m/s",fontsize=14) 
        plt.text(-285, 250, "current curvature = "+str(round(current_info.trajectory_point.kappa,4)),fontsize=14) 
        PlotVehicle(current_info.trajectory_point.x,current_info.trajectory_point.y,current_info.trajectory_point.theta,zoom,0.0)

        # plt.annotate("R=80",xy=(164.4,13.47), xycoords='data',xytext= (-60,+60),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        # plt.annotate("R=40",xy=(188.3,228.2), xycoords='data',xytext= (-80,-80),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        # plt.annotate("R=40",xy=(55.43,88.43), xycoords='data',xytext= (+40,+40),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        # plt.annotate("R=60",xy=(26.39,218.1), xycoords='data',xytext= (-100,-60),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        # plt.annotate("R=60",xy=(-247.8,216.3), xycoords='data',xytext= (+60,-60),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        # plt.annotate("R=20",xy=(-184.3,112.4), xycoords='data',xytext= (+40,+40),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        # plt.annotate("R=40",xy=(-236.1,22.73), xycoords='data',xytext= (+40,+40),textcoords = 'offset points'
        #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))




        plt.legend()
        plt.xlabel('X/m')
        plt.ylabel('Y/m')
        axes = plt.gca()
        axes.set_xlim([xmin,xmax])
        axes.set_ylim([ymin,ymax])

        plt.pause(0.01)
        plt.cla()

def PlotVehicle(x, y, yaw, zoom, steer=0.0, truck_color="-k"):  # pragma: no cover

    # Vehicle parameters
    LENGTH = 4*zoom  # [m]
    WIDTH = 2*zoom   # [m]
    BACK_TO_WHEEL = 1*zoom   # [m]
    WHEEL_LEN = 0.3*zoom   # [m]
    WHEEL_WIDTH = 0.2*zoom   # [m]
    TREAD = 0.7*zoom   # [m]
    WB = 2.86*zoom 

    outline = np.array(
        [[-BACK_TO_WHEEL, (LENGTH - BACK_TO_WHEEL), (LENGTH - BACK_TO_WHEEL),
          -BACK_TO_WHEEL, -BACK_TO_WHEEL],
         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array(
        [[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH -
          TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[cos(yaw), sin(yaw)],
                     [-sin(yaw), cos(yaw)]])
    Rot2 = np.array([[cos(steer), sin(steer)],
                     [-sin(steer), cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truck_color)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truck_color)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truck_color)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truck_color)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truck_color)
    plt.plot(x, y, "*")




if __name__ == "__main__":
    rospy.init_node('trajectory_plot')
    rospy.sleep(3)
    TrajectoryPlotter()
