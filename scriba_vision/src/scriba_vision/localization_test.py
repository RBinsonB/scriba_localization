#!/usr/bin/env python
import rospy, rospkg
from scriba_vision.image_localization import ImageLocalization
import cv2
import numpy as np
import os, sys, math
from scriba_vision.exceptions import ImageLocalizationError

picture_path = os.path.join(rospkg.RosPack().get_path('scriba_vision'), 'ressources/')
textmap = cv2.imread(picture_path+'textmap2.png')

localizer = ImageLocalization(textmap)
axis_length = 100
output = textmap.copy()

try:
    # Try with a lookup window
    (x, y, theta) = localizer.localize(cv2.imread(picture_path+'saved_picture_2.png'), [np.int32([[0, 0], [0, 500], [900, 500], [900, 0]]).reshape(-1,1,2)])
    print(x, y, theta)

    # Draw the coordinate frame
    # x
    cv2.arrowedLine(output,
                    (int(x),int(y)),
                    (int(x + axis_length*math.cos(theta)), int(y - axis_length*math.sin(theta) ) ),
                    (0, 0, 255), 3, cv2.LINE_AA)
    # y
    cv2.arrowedLine(output,
                    (int(x),int(y)),
                    (int(x + axis_length*math.sin(theta)), int(y+ axis_length*math.cos(theta))),
                    (0, 255, 0), 3, cv2.LINE_AA)

    # Try another lookup window
    (x2, y2, theta2) = localizer.localize(cv2.imread(picture_path+'saved_picture_2.png'), [np.int32([[800, 120], [800, 600], [1600, 600], [1600, 120]]).reshape(-1,1,2)])
    print(x2, y2, theta2)


except ImageLocalizationError as e:
    print(e)

cv2.imshow("localization", output)
cv2.waitKey(0)