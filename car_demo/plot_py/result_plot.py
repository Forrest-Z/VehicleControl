import numpy as np
import matplotlib.pyplot as plt

desired_trajectory = '/home/ht/ControlModule/src/car_demo/test_trajectories/mat_trajectory_10.txt'
actual_trajectory = 'simulation_result.txt'

DESIRED_TRAJECTORY_DATA_NUM = 10
""" trajectory_point{
  x: 0.000000
  y: 0.000000
  z: 1.000000
  theta: 0.000000
  kappa: 0.000000
  s: 0.000000
  dkappa: 0.000000
  v: 15.000000
  a: 0.000000
  relative_time: 0.000000
} """

ACTUAL_TRAJECTORY_DATA_NUM = 48
""" point_data{
  time: 1342.97
  x: -0.00689148
  y: 3.33248e-05
  velocity: 0.00698189
  acceleration: 0.698189
  heading: 0.00034535
  relative_time: 1342.99
  distance_error: 0
  velocity_error: 0
  heading_error: 0
  goal_x: 0
  goal_y: 0
  preview_x: 0
  preview_y: 0
  x_from_chassis: -0.00682411
  y_from_chassis: -8.71993e-06
  z_from_chassis: 0.506902
  heading_from_chassis: 3.06826
  vel_from_localization: 0.0174102
  vel_from_wheels: -0.418645
  acc_from_wheels: 0.0280863
  travel_distance: 0.546946
  steering_wheel_angle_actual: 5.89225e-07
  steering_wheel_expected: 0
  steering_wheel_error: 5.89225e-07
  steering_wheel_cmd: -0.0117877
  fl_steering_angle_actual: 0.0035063
  fr_steering_angle_actual: 0.00340317
  single_track_steering_angle: 0
  fl_steering_angle_expected: 0
  fr_steering_angle_expected: 0
  fl_steering_error: 0.0035063
  fl_steering_cmd: -2138.61
  fr_steering_error: 0.00340317
  fr_steering_cmd: -2075.74
  fl_wheel_angular_velocity: -0.216574
  fr_wheel_angular_velocity: -0.20965
  bl_wheel_angular_velocity: -0.0199113
  br_wheel_angular_velocity: -0.0199261
  gas_percent: 0.0994436
  fl_gas_torque: 85.4619
  fr_gas_torque: 85.4619
  bl_gas_torque: 0
  br_gas_torque: 0
  fl_brake_Torque: 1031.28
  fr_brake_torque: 1031.28
  bl_brake_torque: 687.52
  br_brake_torque: 687.52
} """

ERROR = 15.3

def read_file(file_name, data_num):
    result = np.zeros((data_num,1),dtype=float)
    with open(file_name, 'r') as fp:
        while True:
            s = fp.readline()
            if s == '':
                break
            if s[-2]=="{":
                temp = np.zeros((data_num,1),dtype=float)
                i=0
                while True:
                    s = fp.readline()
                    index = s.find(":")
                    if(index!=-1):
                        ss=float(s[index+2:-1:1])
                        temp[i,0]=ss
                        i +=1
                    if s[0]=="}":
                        break
                result = np.hstack([result,temp])
    return result



desired_trajectory=read_file(desired_trajectory,DESIRED_TRAJECTORY_DATA_NUM)
actual_trajectory=read_file(actual_trajectory,ACTUAL_TRAJECTORY_DATA_NUM)


initial_time = actual_trajectory[0,1]

plt.figure(1)
plt.title('trajectory comparison') 
plt.plot(desired_trajectory[0,1:-1], desired_trajectory[1,1:-1], color='cyan', label='desired_trajectory')
plt.plot(actual_trajectory[1,1:-1], actual_trajectory[2,1:-1], 'b', label='actual_trajectory')
plt.legend()
plt.xlabel('X/m')
plt.ylabel('Y/m')
plt.grid()  

plt.figure(2)
plt.title('velocity comparison with time') 
plt.plot(desired_trajectory[9,1:-1], desired_trajectory[7,1:-1], color='cyan', label='desired_trajectory')
plt.plot(actual_trajectory[0,1:-1]-initial_time-13 , actual_trajectory[3,1:-1], 'b', label='actual_trajectory')
plt.legend()
plt.xlabel('T/s')
plt.ylabel('Velocity/(m/s)')
plt.grid()

plt.figure(3)
plt.title('yaw angle comparison with time') 
plt.plot(desired_trajectory[9,1:-1], desired_trajectory[3,1:-1], color='cyan', label='desired_trajectory')
plt.plot(actual_trajectory[0,1:-1]-initial_time-13, actual_trajectory[5,1:-1], 'b', label='actual_trajectory')
plt.legend()
plt.xlabel('T/s')
plt.ylabel('yaw angle')
plt.grid()

plt.figure(4)
plt.title('velocity comparison with station') 
plt.plot(desired_trajectory[5,1:-1], desired_trajectory[7,1:-1], color='cyan', label='desired_trajectory')
plt.plot(actual_trajectory[21,1:-1] , actual_trajectory[3,1:-1], 'b', label='actual_trajectory')
plt.legend()
plt.xlabel('S/m')
plt.ylabel('Velocity/(m/s)')
plt.grid()

plt.figure(5)
plt.title('yaw angle comparison with station') 
plt.plot(desired_trajectory[5,1:-1], desired_trajectory[3,1:-1], color='cyan', label='desired_trajectory')
plt.plot(actual_trajectory[21,1:-1], actual_trajectory[5,1:-1], 'b', label='actual_trajectory')
plt.legend()
plt.xlabel('S/m')
plt.ylabel('yaw angle')
plt.grid()

plt.show()



        
        
        
