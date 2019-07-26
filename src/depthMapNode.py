#!/usr/bin/env python

import roslib
roslib.load_manifest('terrain_characterizer')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import time

from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score

from matplotlib import pyplot as plt

import pandas as pd

import scipy
from scipy.misc import bytescale

from sklearn.decomposition import PCA

program_starts = time.time()
counter = 0

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/dyret/sensor/camera/depth",Image,self.callback)

  def callback(self,data):
    global counter

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e)

    (rows,cols) = cv_image.shape

    width = 300
    height = 400
    step = 2
    
    # Crop image
    cv_image_cropped = cv_image[rows/2 - height/2 : rows/2 + height/2 : step, cols/2 - width/2 : cols/2 + width/2 : step]
    cv_image_cropped[cv_image_cropped == 65535] = 0

    points = []
    distances = []

    pca_points = []

    (rows, cols) = cv_image_cropped.shape
    for i in range(rows):
      for j in range(cols):
        if cv_image_cropped[i][j] > 0:
          points.append([i, j])
          distances.append(cv_image_cropped[i][j])

          pca_points.append([i, j, cv_image_cropped[i][j]])

    print("Max distance: {}, number of points: {}".format(np.max(distances), len(distances)))

    # Create linear regression object
    regr = linear_model.LinearRegression(fit_intercept=True)

    # Train the model using the training sets
    regr.fit(points, distances)

    points_pred = regr.predict(points)

    # The coefficients
    print('Coefficients: {}'.format(regr.coef_))
    print('Intercept: {}'.format(regr.intercept_))
    print("Mean squared error: %.2f" % mean_squared_error(points_pred, distances))

    now = time.time()
    counter += 1
    #print("{0} image per second".format(counter / (now - program_starts)))

    print

    # Make
    cv_image_normalized = cv_image_cropped
    cv_image_normalized[cv_image_normalized != 0] = np.subtract(cv_image_normalized[cv_image_normalized != 0],
                                                                np.min(cv_image_normalized[cv_image_normalized != 0]))
    cv_image_normalized = np.floor_divide(cv_image_normalized, np.max(cv_image_normalized) / 255.0)

    # Convert to color and display
    cv_image__8bit = np.array(cv_image_normalized, dtype=np.dtype('uint8'))
    cv_image_colored = cv2.applyColorMap(cv_image__8bit, cv2.COLORMAP_JET)
    cv2.imshow("Image window", cv_image_colored)
    cv2.waitKey(3)

    # Try PCA
    pca = PCA(n_components=3)
    principalComponents = pca.fit_transform(pca_points)

    print(principalComponents.shape)

    print(pca.explained_variance_ratio_)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
