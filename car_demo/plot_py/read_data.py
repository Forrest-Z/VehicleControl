import numpy as np
import matplotlib.pyplot as plt
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs import String
from nav_msgs import Odometry
from sensor_msgs import NavSatFix
from sensor_msgs import Imu
from dateutil.parser import parse

IMU_DATA_NUM = 11
VEHICLE_STATE_NUM = 14
GPS_DATA_NUM = 5
CHASSIS_DATA_NUM = 9

FILE_NAME= "2020-07-08T12:27:01.txt"
global INITIAL_TIME_STAMP


def find_all(s, char):
    temp = []
    index = 0
    for i in s:
        if char == i:
            temp.append(index)
        index+=1
    return temp

def get_data(s,container,size):
    temp = np.zeros(size,dtype=float)
    pos = find_all(s,"\t")
    length = len(pos)
    for i in range(length-1):
        temp[i]=float(s[pos[i]+1:pos[i+1]])
    temp[-2]=float(s[pos[-1]:-1])
    pos_INFO = s.find("INFO")
    time_str = s[11:pos_INFO-1]
    time_stamp = parse(time_str)
    time_interval = time_stamp-initial_time_stamp
    temp[-1]=time_interval.total_seconds()

    container = np.vstack([container,temp])
    return container


def read_file(file_name):
    IMU = np.zeros(IMU_DATA_NUM,dtype=float)
    vehicle_state = np.zeros(VEHICLE_STATE_NUM,dtype=float)
    gps_data = np.zeros(GPS_DATA_NUM,dtype=float)
    chassis = np.zeros(CHASSIS_DATA_NUM,dtype=float)
    with open(file_name, 'r') as fp:
        s = fp.readline()
        pos_INFO = s.find("INFO")
        time_str = s[11:pos_INFO-1]
        global initial_time_stamp
        initial_time_stamp= parse(time_str)
        while True:
            s = fp.readline()
            if s == '':
                break
            else:
                index_1 = s.find("@")
                index_2 = s.find("]",index_1)
                sign = s[index_1+1:index_2]
                if(sign=="61"):
                    IMU = get_data(s,IMU,IMU_DATA_NUM)
                elif (sign=="101"):
                    vehicle_state=get_data(s,vehicle_state,VEHICLE_STATE_NUM)
                elif (sign=="132"):
                    gps_data=get_data(s,gps_data,GPS_DATA_NUM)
                elif (sign=="141"):
                    chassis=get_data(s,chassis,CHASSIS_DATA_NUM)
    DATA = [IMU,vehicle_state,gps_data,chassis]
    return DATA

data = read_file(FILE_NAME)
print(data[0])



# IMU
imu_msg = Imu()
imu_msg.orientation.x = data[0][1:-1,0]
imu_msg.orientation.y = data[0][1:-1,1]
imu_msg.orientation.z = data[0][1:-1,2]
imu_msg.orientation.w = data[0][1:-1,3]
imu_msg.linear_acceleration.x=data[0][1:-1,4]
imu_msg.linear_acceleration.y=data[0][1:-1,5]
imu_msg.linear_acceleration.z=data[0][1:-1,6]
imu_msg.angular_velocity.x=data[0][1:-1,7] 
imu_msg.angular_velocity.y=data[0][1:-1,8] 
imu_msg.angular_velocity.z=data[0][1:-1,9] 

# vehicle_state
vehicle_msg =Odometry()
vehicle_msg.pose.pose.pose.position.x =data[1][1:-1,0]
vehicle_msg.pose.pose.pose.position.y =data[1][1:-1,1]
vehicle_msg.pose.pose.pose.position.z =data[1][1:-1,2]
vehicle_msg.pose.pose.orientation.x = data[1][1:-1,3]
vehicle_msg.pose.pose.orientation.y = data[1][1:-1,4]
vehicle_msg.pose.pose.orientation.z = data[1][1:-1,5]
vehicle_msg.pose.pose.orientation.w = data[1][1:-1,6] 
vehicle_msg.twist.twist.linear.x= data[1][1:-1,7]
vehicle_msg.twist.twist.linear.y= data[1][1:-1,8]
vehicle_msg.twist.twist.linear.z= data[1][1:-1,9]


# GPS
pos_gps = NavSatFix()
pos_gps.latitude = data[2][1:-1,1]
pos_gps.longitude = data[2][1:-1,2]
pos_gps.altitude = data[2][1:-1,3]

# chassis
chassis_msg.Steer = data[3][1:-1,0]
chassis_msg.Throttle = data[3][1:-1,1]
chassis_msg.Brake = data[3][1:-1,2]
chassis_msg.Wheel = data[3][1:-1,3]
chassis_msg.Speed = data[3][1:-1,4]
chassis_msg.BucketAngle = data[3][1:-1,5]
chassis_msg.EngineSpeed = data[3][1:-1,6]
chassis_msg.EngineTorque = data[3][1:-1,7]


plt.figure(1)
plt.title('trajectory') 
plt.plot(pos_gps.latitude,pos_gps.longitude, color='cyan', label='desired_trajectory')
plt.xlabel('latitude')
plt.ylabel('longitude')
plt.show()


