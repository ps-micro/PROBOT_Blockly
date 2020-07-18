#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Erle Robotics LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import time
import os
import threading
import signal
import rosnode
import mavros

from subprocess import Popen
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from autobahn.asyncio.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
from probot_blockly.srv import SetCurrentBlockId, SetCurrentBlockIdResponse
from std_msgs.msg import String
from sensor_msgs.msg import Joy
#from crab_msgs.msg import apm_imu
#from crab_msgs.msg import BodyCommand
#from crab_msgs.msg import BodyState
#from crab_msgs.msg import GaitCommand
#from crab_msgs.msg import LegIKRequest
#from crab_msgs.msg import LegJointsState
#from crab_msgs.msg import LegPositionState
#from crab_msgs.msg import LegsJointsState
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

try:
    import asyncio
except ImportError:
    # Trollius >= 0.3 was renamed
    import trollius as asyncio


class CodeStatus(object):
    RUNNING = 'running'
    PAUSED = 'paused'
    COMPLETED = 'completed'

    __current_status = COMPLETED
    __write_lock = threading.Lock()
    __status_publisher = None

    @classmethod
    def initialize_publisher(cls):
        if cls.__status_publisher is None:
            cls.__status_publisher = rospy.Publisher('current_code_status', String, queue_size=5)

    @classmethod
    def get_current_status(cls):
        return cls.__current_status

    @classmethod
    def set_current_status(cls, value):
        with cls.__write_lock:
            if value not in [cls.RUNNING, cls.PAUSED, cls.COMPLETED]:
                raise Exception('Incorrect status of code: ' + value)
            else:
                cls.__current_status = value
            cls.initialize_publisher()
        cls.__status_publisher.publish(value)


class CodeExecution(object):
    __node_process = None
    __run_lock = threading.Lock()

    @classmethod
    def run_process(cls, arguments):
        with cls.__run_lock:
            if cls.__node_process is not None:
                rate = rospy.Rate(5)
                for _ in range(10):
                    if cls.__node_process.poll() is None:
                        rate.sleep()
                    else:
                        break
                if cls.__node_process.poll() is None:
                    cls.__node_process.terminate()
            cls.__node_process = Popen(arguments)
            global pid
            pid = cls.__node_process.pid
            # print("\n######################")
            # print("Program's PID="+str(pid))
            # print("######################\n")


class BlocklyServerProtocol(WebSocketServerProtocol):
    __current_code_status_subscriber = None
    __current_block_id_subscriber = None
    __current_block_publisher = None

    def _send_code_status(self, message):
        rospy.loginfo('Current code status: %s', message.data)
        payload = 'status_update\n'
        payload += message.data
        try:
            self.sendMessage(payload.encode('utf-8'), False)
        except Exception as e:
            pass

    def _send_current_block_id(self, message):
        payload = 'set_current_block\n'
        payload += message.data
        try:
            self.sendMessage(payload.encode('utf-8'), False)
        except Exception as e:
            pass

    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))
        if self.__current_code_status_subscriber is None:
            rospy.Subscriber('current_code_status', String, self._send_code_status)
        if self.__current_block_id_subscriber is None:
            rospy.Subscriber('current_block_id', String, self._send_current_block_id)

    def onOpen(self):
        print("WebSocket connection open.")

    def onMessage(self, payload, isBinary):
        # Debug
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            # print("Text message received: {0}".format(payload.decode('utf8')))

            # Do stuff
            # pub = rospy.Publisher('blockly', String, queue_size=10)
            # time.sleep(1)
            # pub.publish("blockly says: "+payload.decode('utf8'))

            # Simple text protocol for communication
            # first line is the name of the method
            # next lines are body of message
            message_text = payload.decode('utf8')
            message_data = message_text.split('\n', 1)

            if len(message_data) > 0:
                method_name = message_data[0]
                if len(message_data) > 1:
                    method_body = message_data[1]
                    if method_name.startswith('play'):
                        CodeStatus.set_current_status(CodeStatus.RUNNING)
                        BlocklyServerProtocol.build_ros_node(method_body)
                        rospy.loginfo('The program source file ['+ os.getcwd() + '/program.py] generated!')
                        # os.system('cat program.py')
                        if method_name == 'play2':
                            CodeExecution.run_process(['python', 'program.py'])
                        elif method_name == 'play3': # But py3 cannot be used
                            CodeExecution.run_process(['python3', 'program.py'])
                    else:
                        rospy.logerr('Called unknown method %s', method_name)
                else:
                    if 'pause' == method_name:
                        CodeStatus.set_current_status(CodeStatus.PAUSED)
                    elif 'resume' == method_name:
                        CodeStatus.set_current_status(CodeStatus.RUNNING)
                    elif 'end' == method_name:
                        #End program.py execution
                        global pid
                        print("@@@@@@@@@@@@@@@@@@")
                        try:
                            print("kill pid="+str(pid))
                            os.kill(pid, signal.SIGKILL)
                            ros_nodes = rosnode.get_node_names()
                        except NameError:
                            print("execution script not running.")
                            pass

                        if '/imu_talker' in ros_nodes: #brain
                            ##set default values
                            pub = rospy.Publisher('/statusleds', String, queue_size=10, latch=True)
                            msg = 'blue_off'
                            pub.publish(msg)
                            msg = 'orange_off'
                            pub.publish(msg)
                        if '/crab_leg_kinematics' in ros_nodes: #spider
                            print("spider running")
                            pub = rospy.Publisher('/joy', Joy, queue_size=10, latch=True)
                            msg = Joy()
                            msg.header.stamp = rospy.Time.now()
                            rate = rospy.Rate(10)
                            valueAxe = 0.0
                            valueButton = 0
                            for i in range (0, 20):
                                msg.axes.append(valueAxe)
                            for e in range (0, 17):
                                msg.buttons.append(valueButton)
                            pub.publish(msg)
                            print("DEFAULT MESSAGES SENT")
                        if '/mavros' in ros_nodes: #rover
                            print("rover")
                            pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
                            msg = OverrideRCIn()
                            msg.channels[0] = 1500
                            msg.channels[1] = 0
                            msg.channels[2] = 1500
                            msg.channels[3] = 0
                            msg.channels[4] = 0
                            msg.channels[5] = 0
                            msg.channels[6] = 0
                            msg.channels[7] = 0
                            pub.publish(msg)
                        print("@@@@@@@@@@@@@@@@@@")

                    elif method_name.startswith('control'):
                        robot = method_name.split('control_')[1]
                        if robot.startswith('spider'):
                            direction = robot.split('spider_')[1]
                            pub = rospy.Publisher('/joy', Joy, queue_size=10)
                            msg = Joy()
                            msg.header.stamp = rospy.Time.now()
                            rate = rospy.Rate(10)
                               
                            valueAxe = 0.0
                            valueButton = 0
                            for i in range (0, 20):
                                msg.axes.append(valueAxe)
                            for e in range (0, 17):
                                msg.buttons.append(valueButton)

                            if direction == 'up':#forward
                                msg.axes[1] = 1
                            elif direction == 'down':#backwards
                                msg.axes[1] = -1
                            elif direction == 'left':#turn left
                                #msg.axes[0] = 1
                                msg.axes[2] = 1
                            elif direction == 'right':#turn rigth
                                #msg.axes[0] = -1
                                msg.axes[2] = -1

                            pub.publish(msg)
                            rate.sleep()

                        elif robot.startswith('rover'):
                            direction = robot.split('rover_')[1]   
                            rospy.wait_for_service('/mavros/set_mode')
                            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                            resp1 = change_mode(custom_mode='manual')
                            print (resp1)
                            if 'True' in str(resp1):
                                pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
                                r = rospy.Rate(10) #2hz
                                msg = OverrideRCIn()
                                throttle_channel=2
                                steer_channel=0

                                speed_slow = 1558
                                #speed_turbo = 2000 #dangerous
                                speed = speed_slow

                                if direction == 'up':#forward
                                    msg.channels[throttle_channel]=speed
                                    msg.channels[steer_channel]=1385 #straight
                                elif direction == 'down':#backwards
                                    msg.channels[throttle_channel]=1450 #slow
                                    msg.channels[steer_channel]=1385 #straight
                                elif direction == 'left':#turn left
                                    msg.channels[throttle_channel]=speed
                                    msg.channels[steer_channel]=1285
                                elif direction == 'right':#turn rigth
                                    msg.channels[throttle_channel]=speed
                                    msg.channels[steer_channel]=1485

                                start = time.time()
                                flag=True
                                while not rospy.is_shutdown() and flag:
                                    sample_time=time.time()
                                    if ((sample_time - start) > 0.5):
                                        flag=False
                                    pub.publish(msg)
                                    r.sleep()
                    else:
                        rospy.logerr('Called unknown method %s', method_name)

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))

    @staticmethod
    def build_ros_node(blockly_code):
        print("Creating the program.py...")
        programFile = open("program.py", 'w')
        programFile.truncate()

        programFile.write('''#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, rosnode, subprocess, os, sys
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from probot_blockly.srv import SetCurrentBlockId

rospy.init_node('blockly_node', anonymous=True)
ros_initial_nodes = rosnode.get_node_names()
probot_initialized = 0

def check_status(block_id):
    rospy.wait_for_service('program_is_paused')
    rospy.wait_for_service('program_set_current_block_id')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            program_is_paused = rospy.ServiceProxy('program_is_paused', Trigger)
            is_paused = program_is_paused()
        except rospy.ServiceException as e:
            print ('Service call failed: ', e)

        if is_paused.success:
            r.sleep()
        else:
            break
    set_current_block = rospy.ServiceProxy('program_set_current_block_id', SetCurrentBlockId)
    set_current_block(block_id)
def send_status_completed():
    rospy.wait_for_service('program_completed')
    try:
        program_completed = rospy.ServiceProxy('program_completed', Empty)
        program_completed()
    except rospy.ServiceException as e:
        print ('Service call failed: ', e)
            ''')
        # Write the code that comes from blockly
        programFile.write("\n" + blockly_code)
        programFile.write('''

ros_final_nodes = rosnode.get_node_names()
S_initial = set(ros_initial_nodes)
S_final = set(ros_final_nodes)
new_nodes = S_initial.symmetric_difference(S_final)
rosnode.kill_nodes(new_nodes)
rosnode.rosnode_cleanup()
        ''')
        programFile.close()


class RobotBlocklyBackend(object):
    __current_block_publisher = None

    @staticmethod
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    @staticmethod
    def __set_status_completed(request):
        CodeStatus.set_current_status(CodeStatus.COMPLETED)
        return EmptyResponse()

    def __set_current_block_id(self, request):
        self.__current_block_publisher.publish(request.block_id)
        response = SetCurrentBlockIdResponse()
        response.result = True
        return response

    @staticmethod
    def __is_status_paused(request):
        response = TriggerResponse()
        response.success = (CodeStatus.PAUSED == CodeStatus.get_current_status())
        return response

    @staticmethod
    def wait_until_ros_node_shutdown(loop):
        while not rospy.is_shutdown():
            time.sleep(.1)
            yield
        loop.stop()

    def talker(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'talker' node so that multiple talkers can
        # run simultaneously.
        rospy.init_node('blockly_server', anonymous=True)
        rospy.Subscriber("blockly", String, RobotBlocklyBackend.callback)
        CodeStatus.initialize_publisher()
        self.__current_block_publisher = rospy.Publisher('current_block_id', String, queue_size=5)

        rospy.Service('program_is_paused', Trigger, RobotBlocklyBackend.__is_status_paused)
        rospy.Service('program_completed', Empty, RobotBlocklyBackend.__set_status_completed)
        rospy.Service('program_set_current_block_id', SetCurrentBlockId, self.__set_current_block_id)

        factory = WebSocketServerFactory(u"ws://0.0.0.0:9000")
        factory.protocol = BlocklyServerProtocol

        loop = asyncio.get_event_loop()
        coro = loop.create_server(factory, '0.0.0.0', 9000)
        server = loop.run_until_complete(coro)
        asyncio.async(RobotBlocklyBackend.wait_until_ros_node_shutdown(loop))

        loop.run_forever()

        print("Closing...")
        server.close()
        loop.run_until_complete(server.wait_closed())
        loop.close()

if __name__ == '__main__':
    backend = RobotBlocklyBackend()
    backend.talker()
