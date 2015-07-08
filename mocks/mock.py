#!/usr/bin/env python

from __future__ import print_function
from time import sleep
import random
import sys
import rospy
from rospy import Publisher, init_node

from sensor_msgs.msg import Range

from pandora_data_fusion_msgs.msg import VictimProbabilities
from pandora_sensor_msgs.msg import BatteryMsg, Co2Msg, Temperature
from pandora_sensor_msgs.msg import ThermalMeanMsg, ImuRPY


def random_battery(delay=1):
    print('Starting random battery data...')
    pub = Publisher('/sensors/battery', BatteryMsg)
    msg = BatteryMsg()
    msg.name = ['PSU', 'Motors']
    while not rospy.is_shutdown():
        battery1 = random.randint(18, 25)
        battery2 = random.randint(18, 25)
        msg.voltage = [battery1, battery2]
        sleep(delay)
        pub.publish(msg)


def random_temperatures(delay=1):
    print('Starting random temperatures...')
    pub = Publisher('/cpu/temperature', Temperature)
    msg = Temperature()
    msg.name = ['cpu0', 'cpu1', 'cpu2', 'cpu2']
    while not rospy.is_shutdown():
        msg.temperature = [random.randint(30, 80) for i in range(4)]
        sleep(delay)
        pub.publish(msg)


def random_co2(delay=1):
    print('Starting random battery data...')
    pub = Publisher('/sensors/co2', Co2Msg)
    msg = Co2Msg()
    while not rospy.is_shutdown():
        msg.header = rospy.Header()
        msg.co2_percentage = random.random()
        sleep(delay)
        pub.publish(msg)


def random_sonar(delay=1):
    print('Starting random sonar...')
    pub = Publisher('/sensors/range', Range)
    msg = Range()
    while not rospy.is_shutdown():
        msg.header = rospy.Header(frame_id='right_sonar_frame')
        msg.range = random.random() * 20
        pub.publish(msg)
        sleep(delay)
        msg.header = rospy.Header(frame_id='left_sonar_frame')
        msg.range = random.random() * 20
        pub.publish(msg)


def random_imu(delay=1):
    print('Starting random imu...')
    pub = Publisher('/sensors/imu_rpy', ImuRPY)
    msg = ImuRPY()
    while not rospy.is_shutdown():
        msg.roll = random.random() * 50
        msg.pitch = random.random() * 50
        msg.yaw = random.random() * 50
        pub.publish(msg)
        sleep(delay)


def random_thermal(delay=1):
    print('Starting random thermal data...')
    pub = Publisher('/sensors/thermal', ThermalMeanMsg)
    msg = ThermalMeanMsg()
    while not rospy.is_shutdown():
        msg.header = rospy.Header()
        msg.thermal_mean = random.randint(20, 40)
        sleep(delay)
        pub.publish(msg)


def random_signs_of_life(delay=1):
    print('Starting random signs of life.')
    pub = Publisher('/data_fusion/signs_of_life', VictimProbabilities)
    msg = VictimProbabilities()
    while not rospy.is_shutdown():
        msg.thermal = random.random()
        msg.co2 = random.random()
        msg.sound = random.random()
        msg.motion = random.random()
        msg.visualVictim = random.random()
        msg.hazmat = random.random()
        sleep(delay)
        pub.publish(msg)


if __name__ == '__main__':
    init_node('mock_node', anonymous=True)

    # Select a mock to use.
    selection = sys.argv[1]
    delay = float(sys.argv[2])

    if selection == 'battery':
        random_battery(delay)
    elif selection == 'co2':
        random_co2(delay)
    elif selection == 'thermal':
        random_thermal(delay)
    elif selection == 'temp':
        random_temperatures(delay)
    elif selection == 'sonar':
        random_sonar(delay)
    elif selection == 'imu':
        random_imu(delay)
    elif selection == 'sol':
        random_signs_of_life(delay)
