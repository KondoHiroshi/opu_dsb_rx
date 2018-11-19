#! /usr/bin/env python3

import sys
import time
import threading
import pyinterface
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64


class cpz340816_controller(object):

    def __init__(self):
        self.rate = rospy.get_param('~rate')
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = 'cpz340816'
        self.flag = 1
        self.lock = 0
        self.param_buff = []        

        try:
            self.da = pyinterface.open(340816, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()
    
        ###=== Define Publisher ===###
        self.pub = rospy.Publisher(cpz340816, Float64, queue_size=1, latch = True)
        ###=== Define Subscriber ===###
        self.sub = rospy.Subscriber(cpz340816_rsw0_ch1, Float64, self.set_param) 
        
    def set_param(self, req):
        self.param_buff = req.data
        pass
            
    def output_voltage(self):
        while not rospy.is_shutdown():

            if self.param_buff == []:
                time.sleep(self.rate)
                continue
                
            param_buff = self.param_buff.copy()
            self.param_buff = []
            voltage = param_buff[0]
            self.da.output_voltage(voltage)
            self.pub.publish(voltage)

    def start_thread_ROS(self):
        th = threading.Thread(target=self.output_voltage)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('cpz340816')
    ctrl = cpz340816_controller()
    ctrl.start_thread_ROS()
    rospy.spin()