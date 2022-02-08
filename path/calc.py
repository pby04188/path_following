import os

points = []
f=open("/home/pby04188/robot_ws/install/path_following/share/path_following/path/turtlebot2.txt", 'r')
lines = f.readlines()
for i in lines:
    tmp = i.split()
    point = [0,0,0]
    point[0] = str(float(tmp[0])+5)
    point[1] = str(float(tmp[1])+5)
    point[2] = tmp[2]
    points.append(point)
    
f.close()

new = open("/home/pby04188/robot_ws/src/path_following/path/turtlebot.txt", 'w')
for i in range(len(points)):
    new.writelines([points[i][0],"\t", points[i][1], "\t", points[i][2], "\n"])
    
new.close()