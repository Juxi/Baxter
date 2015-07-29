#!/usr/bin/env python
# 2015 Juxi Leitner <j.leitner@roboticvision.org>
# Australian Centre for Robotic Vision (ACRV)

import cv2
import cv_bridge
import time

################################################################################
class VisualDistance(object):
    """
    """

    def __init__(self, device=0):
        self.is_stopping = False
        self.bridge = cv_bridge.CvBridge()
        self.cam = cv2.VideoCapture(device)

        # Set the blob detector parameters
        target_blob_params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        target_blob_params.minThreshold = 10;
        target_blob_params.maxThreshold = 200;

        # Filter by Colour
        target_blob_params.filterByColor = True
        target_blob_params.blobColor = 255

        # Filter by Area
        # params.filterByArea = True
        # params.minArea = 1500

        # Filter by Circularity
        target_blob_params.filterByCircularity = True
        target_blob_params.minCircularity = 0.1

        # Filter by Convexity
        target_blob_params.filterByConvexity = True
        target_blob_params.minConvexity = 0.8

        self.target_detector = cv2.SimpleBlobDetector(target_blob_params)

        # start the detector
        self.update()



    def update(self):
        """ calculate the camera image/visual distance """
        # perform the calculation of the distance between the end-effector
        # and the target/goal visible in the image
        while not self.is_stopping and self.cam.isOpened():
            # retrive camera image
            ret, frame = self.cam.read()
            if ret == False: break

            # TODO do some operation on the camera image here!
            # detect the end effector in pixel coords

            # detect the target blob position in pixel coords
            keypoints = self.target_detector.detect(im)

            # Draw detected blobs as red circles.
            # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
            # the size of the circle corresponds to the size of blob

            frame_w_keypts = cv2.drawKeypoints(frame, keypoints,
                np.array([]), (0,0,255),
                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Show blobs
            cv2.imshow("Keypoints", frame_w_keypts)

            time.sleep(0.05)

        self.cam.release()


    def clean_shutdown(self):
        # stop the controller and the ROS node
        print("\nExiting Cam Node!")
        self.is_stopping = True


################################################################################
def main():
    # start the node
    arm = VisualDistance()

    # when it returns it is done
    print("Done.")


##########################
if __name__ == '__main__':
    main()
