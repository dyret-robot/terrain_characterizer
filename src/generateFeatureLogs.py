#!/usr/bin/env python

import roslib
roslib.load_manifest('terrain_characterizer')
import sys
import rospy
import subprocess
import ntpath
import progressbar
from terrain_characterizer.srv import *

import os
import os.path

def startLogging(givenServiceProxy, givenLogPath):
    givenServiceProxy(givenLogPath)

def stopLogging(givenServiceProxy):
    givenServiceProxy("")

# Find all bag files
bagFiles = []

for dirpath, dirnames, filenames in os.walk("/home/nyg003/catkin_ws/experimentResults/"):
    for filename in [f for f in filenames if f.endswith(".bag")]:
        bagFiles.append(os.path.join(dirpath, filename))

bagFiles.sort()

for bagFile in progressbar.progressbar(bagFiles):
    bagName = ntpath.basename(bagFile)
    bagDirectory = ntpath.dirname(bagFile)

    # Do service call to setup logging
    rospy.wait_for_service('/dyret/pointCloudPlaneFitter/featureLogging')

    try:
        featureLogging_pointCloud = rospy.ServiceProxy('/dyret/pointCloudPlaneFitter/featureLogging', featureLoggingService)
        featureLogging_optoForce = rospy.ServiceProxy('/dyret/optoforceFeatureExtractor/featureLogging', featureLoggingService)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

    startLogging(featureLogging_pointCloud, bagFile.replace(".bag", "_features_pc.csv"))
    startLogging(featureLogging_optoForce, bagFile.replace(".bag", "_features_f.csv"))

    # Play bag file
    FNULL = open(os.devnull, 'w')
    process = subprocess.Popen(['rosbag', 'play', bagName], cwd=bagDirectory, stdout=FNULL)
    process.wait()

    # Stop logging
    stopLogging(featureLogging_pointCloud)
    stopLogging(featureLogging_optoForce)

