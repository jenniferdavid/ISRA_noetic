#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from temperature_layer_isra.cfg import TemperatureLayerConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request for TEMPERATURE: {lower_threshold}, {lower_threshold_scale},\ 
          {upper_threshold}, {upper_threshold_scale}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("Temperature_Layer_Server", anonymous = False)

    srv = Server(TemperatureLayerConfig, callback)
    rospy.spin()
