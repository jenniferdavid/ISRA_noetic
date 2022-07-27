#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from radiation_layer_isra.cfg import RadiationLayerConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request for RADIATION: {lower_threshold}, {lower_threshold_scale},\ 
          {upper_threshold}, {upper_threshold_scale}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("Radiation_Layer_Server", anonymous = False)

    srv = Server(RadiationLayerConfig, callback)
    rospy.spin()
