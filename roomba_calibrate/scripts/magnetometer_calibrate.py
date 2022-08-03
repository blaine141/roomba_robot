#!/usr/bin/env python3

# Import required Python code.
from math import atan, atan2, pi, sqrt
from typing import List
import rospy
import numpy as np
import tf

# Import custom message data.
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()


final_data = []
compass_msgs: List[Vector3Stamped] = []
fiducial_msgs = []

# Create a callback function for the subscriber.
def magnetometer_callback(data: Vector3Stamped):
    compass_msgs.append(data)
        
def image_callback(data: CompressedImage):
    frame = bridge.compressed_imgmsg_to_cv2(data)
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
		arucoDict, parameters=arucoParams)

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the frame
            cv2.putText(frame, str(markerID),
                (topLeft[0], topLeft[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

            camera_matrix = np.array([
                [502.0598599590367, 0, 315.4010495654025],
                [0, 500.1203338481508, 224.0738451287138],
                [0, 0, 1]
            ])

            (rvec, tvec, _) = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.15, camera_matrix, None)

            if tvec[0,0,2] > 2.8:
                return
                
            position = Vector3Stamped()
            position.header = data.header
            position.vector.x, position.vector.y, position.vector.z = tvec[0,0]
            position_pub.publish(position)

            


            rotation_matrix = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 1]],
                            dtype=float)
            rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

            # convert the matrix to a quaternion
            quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

            msg = QuaternionStamped()
            msg.header = data.header
            msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w = quaternion
            quaternion_pub.publish(msg)

            heading = 2 * atan(msg.quaternion.y / msg.quaternion.x) * 180 / pi
            heading_pub.publish(heading)

            fiducial_msgs.append((data.header.stamp, heading))

    msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
    annotated_pub.publish(msg)

            
def flush(_):
    while len(compass_msgs) > 0:
        compass_msg = compass_msgs[0]
        compass_time = compass_msg.header.stamp

        found_heading = False
        for heading_time, heading in fiducial_msgs:
            if heading_time > compass_time:
                found_heading = True
                break
            fiducial_msgs.pop(0)
        
        if not found_heading:
            return

        time_offset: rospy.Duration = heading_time - compass_time
        time_pub.publish(time_offset.to_sec())

        if time_offset.to_sec() < 0.1:
            final_data.append((compass_msg.vector, heading))
        compass_msgs.pop(0)
        



# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('aruco_parser')
    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber("/cv_camera/image_rect/compressed", CompressedImage, image_callback)

    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber("compass_raw", Vector3Stamped, magnetometer_callback)

    # Publisher for output
    annotated_pub = rospy.Publisher("annotated", Image, queue_size=20)
    position_pub = rospy.Publisher("position", Vector3Stamped, queue_size=20)
    quaternion_pub = rospy.Publisher("orientation", QuaternionStamped, queue_size=20)
    heading_pub = rospy.Publisher("heading", Float32, queue_size=20)
    time_pub = rospy.Publisher("measurement_time_offset", Float32, queue_size=20)

    timer = rospy.Timer(rospy.Duration(0.1), flush)

    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

    print("Saving")

    with open("magnetometer_data.csv", 'w') as csv_file:
        for vector, heading in final_data:
            csv_file.write("%f, %f, %f, %f\n" % (vector.x, vector.y, vector.z, heading))

    print("Stopping")