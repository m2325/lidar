import socket
#import math
import numpy as np
#import pandas as pd
#import os
import struct

##画个简单三维图
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
ax = plt.figure().add_subplot(111, projection = '3d')

HOST = "192.168.1.201"
PORT = 2368

LASER_ANGLES = [-30.67, -9.33, -29.33, -8, -28, -6.67, -26.67, -5.33, -25.33, -4, -24, -2.67, -22.67, -1.33, -21.33, 0, -20, 1.33, -18.67, 2.67, -17.33, 4, -16, 5.33, -14.67, 6.67, -13.33, 8, -12, 9.33, -10.67, 10.67]
NUM_LASERS = 32

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002#转换成米
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000


def init_velo_socket():
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('192.168.1.104', PORT))

#data_buff = pd.DataFrame(columns=['x', 'y', 'z', 'distance'])

def calc(dis, azimuth, laser_id):
    R = dis * DISTANCE_RESOLUTION#转换成米
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0#转换成弧度
    alpha = azimuth / 100.0 * np.pi / 180.0#转换成弧度
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    return [X, Y, Z, R]

def get_pointcloud(soc):
    data_buff = []
    data_x = []
    data_y = []
    data_z = []
    count = 0
    timestamp = 0
    time_offset = 0

    while True:
        # get data from port
        data = soc.recv(1248)
        count += 1
        if count == 90:
            count = 0
            break
        #print('pcl size: ', len(data))#data为1206个字节的数据，包含1200个数据与2+4的出厂设置
        #print('data: ',data)
        for offset in range(0, 1200, 100):
            #flag, azimuth = struct.unpack_from("<HH", data, offset)
            flag = struct.unpack_from("<H", data, offset)#H代表unsigned short 2个字节，详见https://docs.python.org/3/library/struct.html#struct.calcsize
            azimuth = struct.unpack_from("<H", data, offset + 2)
            #print('flag: ',flag)
            #print('azimuth: ',azimuth)
            for i in range(NUM_LASERS):
                arr = struct.unpack_from("<H", data, offset + 4 + i * 3)
                #print('arr: ',arr)
                #print('arr[0]: ',arr[0])
                strong = struct.unpack_from('<B', data, offset + 4 + 4 + i * 3)#B代表unsigned char 1个字节
                #print('strong: ',strong)
                if arr[0] != 0:# 经过struct.unpack_from处理的结果为元祖,滤除距离值为0的点
                    x, y, z, dist = calc(arr[0], azimuth[0], i)#x, y, z, dist = calc(arr[i * 2], azimuth, i, timestamp + time_offset)
                    if z > 0:#源程序可能筛选z大于0的值
                     data_buff.append([x, y, z, dist , strong[0]])
                     data_x.append(x)
                     data_y.append(y)
                     data_z.append(z)
#基于ax变量绘制三维图
#xs表示x方向的变量
#ys表示y方向的变量
#zs表示z方向的变量，这三个方向上的变量都可以用list的形式表示
#m表示点的形式，o是圆形的点，^是三角形（marker)
#c表示颜色（color for short）
    ax.scatter(data_x, data_y, data_z, c = 'r', marker = 'o') #点为红色三角形
    return np.array(data_buff)


# Code to test point clouds: 
print('point cloud test')              
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('192.168.1.104', PORT))
np.savetxt('pcl.csv', get_pointcloud(soc), delimiter=',')
soc.close()



#设置坐标轴
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
#显示图像
plt.show()
 
#ax = plt.figure().add_subplot(111, projection = '3d')
# 
#fig = plt.figure()
#ax = fig.add_subplot(111, projection = '3d')
