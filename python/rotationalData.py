#!/usr/bin/python

# Connect to the TP-LINK Network.
# Start the webcam on the IP Webcam app inside of the android phone. 
# Replace the IP_WEBCAM_ADDRESS field in this program with the ip provided in the app.
# Open the stereopi app to view the stereopi camera realtime.

from __future__ import division
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import requests
import json
import time
import os

IP_WEBCAM_ADDRESS = os.environ.get("webcam_ip") or "192.168.0.108:8080"
print(IP_WEBCAM_ADDRESS)

publisher_pantilt_joystick = rospy.Publisher('pantilt_joystick', Vector3, queue_size=3)
rospy.init_node('rotational_data', anonymous=True)
rate = rospy.Rate(10) # 10hz


while not rospy.is_shutdown():
    start = time.time()
    response = requests.get("http://" + IP_WEBCAM_ADDRESS + "/sensors.json", verify=False)
    rotation = Vector3(json.dumps(response.json()['rot_vector']['data'][0][1][0]), json.dumps(response.json()['rot_vector']['data'][0][1][1]), json.dumps(response.json()['rot_vector']['data'][0][1][2]))
    print(time.time()-start)
    rospy.loginfo(rotation)
    publisher_pantilt_joystick.publish(rotation)

    rate.sleep()
    