#!/usr/bin/env python


import pyrealsense2 as rs
import numpy as np
import cv2




class CameraSetup:


    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(config)
        self.crop_start = 0



    def getFrames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data()) ##Access color component of frameset

        coloriser = rs.colorizer()

        align = rs.align(rs.stream.color)
        frameset = align.process(frames)
        aligned_depth_frame = frameset.get_depth_frame()
        colorized_depth = np.asanyarray(coloriser.colorize(aligned_depth_frame).get_data())

        return color_image, colorized_depth, aligned_depth_frame


    def imagePreprocessing(self, color_image, colorized_depth, expectedSize):
        # Pre Processing of images, resize the Images and set the variables
        height, width = color_image.shape[:2]
        aspect = width / height
        self.crop_start = round(expectedSize * (aspect - 1) / 2)
        shrinkImage = cv2.resize(color_image, None, fx = expectedSize/width, fy = expectedSize/height, interpolation=cv2.INTER_AREA)
        shrinkDepth = cv2.resize(colorized_depth, None, fx=expectedSize / width, fy=expectedSize / height,
                                     interpolation=cv2.INTER_AREA)
        return shrinkImage, shrinkDepth

    def stopProcess(self):
        self.pipeline.stop()

class ImageRecognition():

    def __init__(self):
        ## values for DNN
        self.expectedSize = 300
        self.inScaleFactor = 0.007843  ##obtained from the documentation of the DNN
        self.meanVal = 127.53

        # DNN Initialization
        self.net = cv2.dnn.readNetFromCaffe("/home/yusseff/Neobotix/src/rplidar_ros/CameraFiles/MobileNet-SSD/deploy.prototxt",
                                            "/home/yusseff/Neobotix/src/rplidar_ros/CameraFiles/MobileNet-SSD/mobilenet_iter_73000.caffemodel")
        self.classNames = ("bicycle", "bird", "boat",
                           "bottle", "bus", "car", "cat", "chair",
                           "cow", "cup", "diningtable", "dog",
                           "fork", "person", "pottedplant",
                           "laptop", "sofa", "train", "tvmonitor", "microwave", "scissors", "book")

    def classification(self, image):
        blob = cv2.dnn.blobFromImage(image, self.inScaleFactor, (self.expectedSize, self.expectedSize), self.meanVal, False)
        self.net.setInput(blob, "data")
        detections = self.net.forward("detection_out")

        label = detections[0, 0, 0, 1]
        xmin = detections[0, 0, 0, 3]
        ymin = detections[0, 0, 0, 4]
        xmax = detections[0, 0, 0, 5]
        ymax = detections[0, 0, 0, 6]


        className = self.classNames[int(label)]
        listOfValues = (xmin, ymin, xmax, ymax, className)

        cv2.rectangle(image, (int(xmin * self.expectedSize), int(ymin * self.expectedSize)),(int(xmax * self.expectedSize), int(ymax * self.expectedSize)), (255, 255, 255), 2)
        cv2.putText(image, className,(int(xmin * self.expectedSize), int(ymin * self.expectedSize) - 5),cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255))

        return image, listOfValues

    def getDepthData(self, depthImage, aligned_depth_frame, cameraValues, listOfValues):
        height, width = depthImage.shape[:2]
        scale = height / self.expectedSize
        xmin_depth = int((listOfValues[0] * self.expectedSize + cameraValues.crop_start) * scale)
        ymin_depth = int((listOfValues[1] * self.expectedSize) * scale)
        xmax_depth = int((listOfValues[2] * self.expectedSize + cameraValues.crop_start) * scale)
        ymax_depth = int((listOfValues[3] * self.expectedSize) * scale)

        depth = np.asanyarray(aligned_depth_frame.get_data())
        # Crop depth data:
        depth = depth[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)

        # Get data scale from the device and convert to meters
        depth_scale = cameraValues.profile.get_device().first_depth_sensor().get_depth_scale()
        depth = depth * depth_scale
        dist, _, _, _ = cv2.mean(depth)

        return dist


if __name__ == "__main__":
    camera = CameraSetup()
    dnnRecognition = ImageRecognition()

    try:
        while True:
            color_image, colorized_depth, aligned_depth_frame = camera.getFrames()

            shrinkImage, shrinkDepth = camera.imagePreprocessing(color_image, colorized_depth, dnnRecognition.expectedSize)

            output_color, listOfValues = dnnRecognition.classification(shrinkImage)
            dist = dnnRecognition.getDepthData(colorized_depth, aligned_depth_frame, camera, listOfValues)


            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', output_color)  ### use images if you want to stack them // use color_image for only image // use colorized_depth for depth information
            #
            print("Detected a {0} {1:.3} meters away.".format(listOfValues[4], dist))


            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        # Stop streaming
        camera.stopProcess()