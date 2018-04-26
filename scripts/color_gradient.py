#!/usr/bin/python
import math
import rospy
from tuw_airskin_msgs.msg import AirskinColors
from tuw_airskin_msgs.msg import AirskinPressures
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
import sys

NUM_COLORS = 4

min_pressure = [sys.maxint,sys.maxint,sys.maxint,sys.maxint]
max_pressure = [-sys.maxint - 1,-sys.maxint - 1,-sys.maxint - 1,-sys.maxint - 1]
rel_pressure = [0,0,0,0]

def getHeatMapColor(value):
	color = [[0,0,1], [0,1,0], [1,1,0], [1,0,0]]
	# A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.
	fractBetween = 0  # Fraction between "idx1" and "idx2" where our value is.
 
	if(value <= 0):
		idx1 = 0
		idx2 = 0 #accounts for an input <=0
	elif(value >= 1):
		idx1 = NUM_COLORS-1
		idx2 = NUM_COLORS-1    # accounts for an input >=0
	else:
		value = value * (NUM_COLORS-1)        # Will multiply value by 3.
		idx1  = int(math.floor(value))                  # Our desired color will be after this index.
		idx2  = idx1+1                        # ... and before this index (inclusive).
		fractBetween = value - float(idx1)    # Distance between the two indexes (0-1).
	red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0]
	green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1]
	blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2]
	return (red,green,blue)

def callback(data):
	global pub
	global pubtwist
	global min_pressure
	global max_pressure
	global rel_pressure
	colors = AirskinColors()
	colors.ids = data.ids
	for i in range(0,4):
		pressure = data.pressures[i]
		if pressure<min_pressure[i]:
			min_pressure[i] = pressure
		if pressure>max_pressure[i]:
			max_pressure[i] = pressure
		rel = (pressure-min_pressure[i])/float(max_pressure[i]-min_pressure[i]+300)
		rel_pressure[i] = rel_pressure[i]*0.7+rel*0.3
		ret = getHeatMapColor(rel)
		color = ColorRGBA()
		color.a = 1
		color.r = ret[0]
		color.g = ret[1]
		color.b = ret[2]
		colors.colors.append(color)
	pub.publish(colors)
	twist = Twist()
	w = (rel_pressure[1]-rel_pressure[2])
	v = 0
	if(abs(w)>0.1):
		twist.angular.z = w
	if(rel_pressure[1]>0.1 and rel_pressure[2]>0.1):
		v = (rel_pressure[1]+rel_pressure[2])/2
	elif(rel_pressure[0]>0.10):
		v= 0.5
	twist.linear.x = -v*0.4
	pubtwist.publish(twist)
	print str(rel_pressure[0]) + ","+ str(rel_pressure[1])+","+str(rel_pressure[2])+","+str(rel_pressure[3])
	print twist
	
	

if __name__ == "__main__":
	global pub
	global pubtwist
	global controller
	rospy.init_node("airskin_color_calculator",anonymous=True)
	rospy.Subscriber("/p3dx/AirSkinNodelet/airskin_pressures", AirskinPressures, callback)
	pub = rospy.Publisher("/p3dx/AirSkinNodelet/airskin_colors", AirskinColors,queue_size=10)
	pubtwist = rospy.Publisher("/p3dx/cmd_vel",Twist,queue_size=10)
	rospy.spin()
	



