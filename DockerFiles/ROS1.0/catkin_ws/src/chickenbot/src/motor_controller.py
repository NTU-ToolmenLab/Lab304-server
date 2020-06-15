#!/usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod
import rospy
from geometry_msgs.msg import Twist
import smbus


__author__ = "Sam Kuo"
__maintainer__ = "Sam Kuo"
__email__ = "kuo77122@gmail.com"
__version__ = "0.1"


class API():
    __metaclass__ = ABCMeta
    @abstractmethod
    def transform(self, linear_velocity, angular_velocity):
        pass

    @abstractmethod
    def send_cmd(self):
        pass

# class SerialAPI(API):
#     """Provide serial interface to communiting with the arduino motor controller
#     """

class SMBusAPI(API):
    """Provide I2C interface to communiting with the arduino motor controller
    """
    def __init__(self, turning_threshold, speed_threshold, address, bus_idx=1):
        # validate parameters
        self.bus_idx = bus_idx
        if address is None:
            raise ValueError('For SMBUS, device address must be provided')
        self.address = address
        if len(turning_threshold) != 2:
            raise ValueError('Should give turning_threshold two value. (1st_th, 2nd_th)')
        self.turning_threshold = tuple(sorted(turning_threshold))
        if len(speed_threshold) != 2:
            raise ValueError('Should give speed_threshold two value. (1st_th, 2nd_th)')
        self.speed_threshold = tuple(sorted(speed_threshold))
        # SMBus Init.
        self.bus = smbus.SMBus(self.bus_idx)
        rospy.loginfo("Init SMBus({} with device address: {}".format(self.bus_idx, self.address))
        rospy.loginfo("Setting speed_threshold={} and turning_threshold={}".format(speed_threshold, turning_threshold))
        # TODO: check smbus device is exist

    def transform(self, linear_velocity, angular_velocity):
        """ 20202-05-25  by 葉逸新
            從Rpi到Arduino的指令是 ~三位數~ 數字: _ _ _
            分別是： linear x, angular z, speed
            0停, 1前進, 2後退 | 0 直行, 1小左轉, 3大左轉 ,2小右轉, 4大右轉 | 速度 1～3段
            ex:101代表以第一段速度直行
            另外它每個指令會執行兩秒，兩秒內沒收到新指令就車會停住
        """
        cmd = [0, 0, 0]
        # 1st cmd
        if linear_velocity > 0:
            cmd[0] = 1
        elif linear_velocity < 0:
            cmd[0] = 2
        else:
            cmd[0] = 0
        # 2nd cmd
        if abs(angular_velocity) < self.turning_threshold[0]:
            cmd[1] = 0
        else:
            if angular_velocity > 0: 
                if abs(angular_velocity) < self.turning_threshold[1]:
                    cmd[1] = 1
                else: # abs(angular_velocity) > self.turning_threshold[1]:
                    cmd[1] = 3
            else: # angular_velocity < 0
                if abs(angular_velocity) < self.turning_threshold[1]:
                    cmd[1] = 2
                else: # abs(angular_velocity) > self.turning_threshold[1]:
                    cmd[1] = 4
        # 3rd cmd
        if abs(linear_velocity) < self.speed_threshold[0]:
            cmd[2] = 1
        elif abs(linear_velocity) < self.speed_threshold[1]:
            cmd[2] = 2
        else:
            cmd[2] = 3
        rospy.logdebug("Gernerate {} from (linear.x, angular.z) = ({}, {})".format(cmd, linear_velocity, angular_velocity))
        return sum(d * 10**i for i, d in enumerate(cmd[::-1]))  # [1,2,3] -> 123

    def send_cmd(self, cmd):
        rospy.logdebug("Send '{}' to device".format(cmd))
        self.bus.write_byte(self.address, cmd)


class MotorController(object):
    protocol_mapping = {'i2c': SMBusAPI,
                        'smbus': SMBusAPI}

    def __init__(self, cmd_vel_topic, arduino_protocol, **kwargs):
        self.cmd_vel_subscriber = rospy.Subscriber(cmd_vel_topic, Twist, self.callback)
        if self.protocol_mapping.get(arduino_protocol) is None:
            raise Exception("Protocol {} can not be recognized.".format(arduino_protocol))
        # init arduino interface
        self.arduino_api_instance = self.protocol_mapping.get(arduino_protocol)(**kwargs)

    def callback(self, msg):
        rospy.logdebug("Received a /cmd_vel message!")
        rospy.logdebug("Linear Components: [{}, {}, {}]".format(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.logdebug("Angular Components: [{}, {}, {}]".format(msg.angular.x, msg.angular.y, msg.angular.z))
        api_cmd = self.arduino_api_instance.transform(msg.linear.x, msg.angular.z)
        rospy.logdebug("Received api_cmd: {}".format(api_cmd))
        self.arduino_api_instance.send_cmd(api_cmd)


if __name__ == '__main__':
    rospy.init_node('motor_controller')
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
    arduino_protocol = rospy.get_param('~arduino_protocol')
    arduino_address = rospy.get_param('~arduino_addresss', None)
    turning_threshold = rospy.get_param('~turning_threshold')
    speed_threshold = rospy.get_param('~speed_threshold')
    bus_idx = rospy.get_param('~bus_idx', None)
    # rosrun chickenbot motor_controller.py _arduino_protocol:=i2c _arduino_addresss:=4 _turning_threshold:=[1,2] _speed_threshold:=[1,2] _bus_idx:=1
    MotorController(cmd_vel_topic=cmd_vel_topic,
                    arduino_protocol=arduino_protocol,
                    address=arduino_address,
                    turning_threshold=turning_threshold,
                    speed_threshold=speed_threshold,
                    bus_idx=bus_idx)
    rospy.spin()
