import numpy as np
from numpy import ogrid

# odom = np.array([(20, 30, 0)])
# # odom = [20, 20],[30, 30],[0, 0] # x, y, seta
# print("odom_old {0}".format(odom))

# odom = np.append(odom,[[1,2,3]],axis=0)
# odom = np.append(odom,[[3,2,1]],axis=0)
# odom = np.append(odom,[[10,20,30]],axis=0)

# print("odom_new \n {0}".format(odom))
# print("\n")
# print(odom[-1])

# print(np.average(odom[-2][0:2]) - np.average(odom[-1][0:2]))
print(np.tan(46.73570458892837*np.pi/180.) * 100)
magic_num = 46.73570458892837

x = np.arange(0,magic_num,5)
x = np.append(x,[magic_num])
for i in range(1,len(x)):
    x_cut = x[i] - x[i-1]
    
# x = ogrid[5:magic_num:5j]
# print("x: {0}".format(x))
