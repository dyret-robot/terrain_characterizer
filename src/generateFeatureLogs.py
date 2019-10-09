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

def generateLogFiles(givenBagFiles, optoforce=True):
	for bagFile in progressbar.progressbar(bagFiles):
		    bagName = ntpath.basename(bagFile)
		    bagDirectory = ntpath.dirname(bagFile)

		    # Do service call to setup logging
		    rospy.wait_for_service('/dyret/pointCloudPlaneFitter/featureLogging')

		    try:
			featureLogging_pointCloud = rospy.ServiceProxy('/dyret/pointCloudPlaneFitter/featureLogging', featureLoggingService)
			if optoforce:
				featureLogging_optoForce = rospy.ServiceProxy('/dyret/optoforceFeatureExtractor/featureLogging', featureLoggingService)
		    except rospy.ServiceException, e:
			print("Service call failed: %s"%e)

		    startLogging(featureLogging_pointCloud, bagFile.replace(".bag", "_features_pc.csv"))
		    if optoforce:
                        startLogging(featureLogging_optoForce, bagFile.replace(".bag", "_features_f.csv"))

		    # Play bag file
		    FNULL = open(os.devnull, 'w')
		    process = subprocess.Popen(['rosbag', 'play', bagName], cwd=bagDirectory, stdout=FNULL)
		    process.wait()

		    # Stop logging
		    stopLogging(featureLogging_pointCloud)
		    if optoforce:
		        stopLogging(featureLogging_optoForce)

# Find all bag files
bagFiles = []

for dirpath, dirnames, filenames in os.walk("/home/tonnesfn/catkin_ws/experimentResults/sensors/"):
    for filename in [f for f in filenames if f.endswith(".bag")]:
        bagFiles.append(os.path.join(dirpath, filename))

bagFiles.sort()

generateLogFiles(bagFiles, False)

bagFiles = []

for dirpath, dirnames, filenames in os.walk("/home/tonnesfn/catkin_ws/experimentResults/phase1"):
    for filename in [f for f in filenames if f.endswith(".bag")]:
        bagFiles.append(os.path.join(dirpath, filename))

bagFiles.sort()

#generateLogFiles(bagFiles, True)
