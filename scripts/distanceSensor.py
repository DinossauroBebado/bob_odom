#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray
from time import sleep

loop_hz = 10
NUMBER_OF_READING = 3
SERVO_ANGLE = {0: 0, 1: 55, 2: 120}

sensorReadMsg = Float32MultiArray()
sensorRead = [0, 0, 0]

servoPub = rospy.Publisher("/bob/ir_motor", UInt16, queue_size=10)
distanceReadingsPub = rospy.Publisher(
    "/bob/distances", Float32MultiArray, queue_size=10)


def readSensorCB(msg):
    global currentReading
    currentReading = msg.data


if __name__ == '__main__':
    rospy.init_node('distanceSensorNode')
    rospy.Subscriber("/bob/ir_sensor", Float32, readSensorCB)
    rate = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        for readings in range(NUMBER_OF_READING):
            # send servo to position
            servoPub.publish(SERVO_ANGLE[readings])
            sleep(1)
            sensorRead[readings] = currentReading
        sensorReadMsg.data = sensorRead
        print(sensorRead)
        distanceReadingsPub.publish(sensorReadMsg)
        rate.sleep()
