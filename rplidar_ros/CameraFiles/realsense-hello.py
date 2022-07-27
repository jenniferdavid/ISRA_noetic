###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue


        # Convert images to numpy arrays
        # depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data()) ##Access color component of frameset

        # plt.rcParams["axes.grid"] = False
        # plt.rcParams['figure.figsize'] = [12, 6]
        # plt.imshow(color_image)
        # cv2.imshow('RealSense', color_image)


        ##capture depth map
        coloriser = rs.colorizer()
        # colorized_depth = np.asanyarray(coloriser.colorize(depth_frame).get_data())
        # cv2.imshow('RealSense', colorized_depth) ### if I only want to use the depth information

        align = rs.align(rs.stream.color)
        frameset = align.process(frames)
        aligned_depth_frame = frameset.get_depth_frame()
        colorized_depth = np.asanyarray(coloriser.colorize(aligned_depth_frame).get_data())

        # flag = True
        # while flag:
        #     key = cv2.waitKey(1)
        #     if key & 0xFF == ord('q') or key == 27:
        #         cv2.destroyAllWindows()
        #         flag = False

        # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #
        # # Stack both images horizontally
        images = np.hstack((color_image, colorized_depth))
        #
        # # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', color_image)  ### use images if you want to stack them // use color_image for only image // use colorized_depth for depth information

        # Standard OpenCV boilerplate for running the net:
        height, width = color_image.shape[:2]
        expected = 300
        aspect = width / height
        resized_image = cv2.resize(color_image, (round(expected * aspect), expected))
        crop_start = round(expected * (aspect - 1) / 2)
        crop_img = resized_image[0:expected, crop_start:crop_start + expected]

        net = cv2.dnn.readNetFromCaffe("../MobileNet-SSD/deploy.prototxt", "../MobileNet-SSD/mobilenet_iter_73000.caffemodel")
        inScaleFactor = 0.007843
        meanVal = 127.53
        classNames = ("background", "aeroplane", "bicycle", "bird", "boat",
                      "bottle", "bus", "car", "cat", "chair",
                      "cow", "diningtable", "dog", "horse",
                      "motorbike", "person", "pottedplant",
                      "sheep", "sofa", "train", "tvmonitor")
        #
        blob = cv2.dnn.blobFromImage(color_image, inScaleFactor, (expected, expected), meanVal, False)
        net.setInput(blob, "data")
        detections = net.forward("detection_out")

        label = detections[0, 0, 0, 1]
        conf = detections[0, 0, 0, 2]
        xmin = detections[0, 0, 0, 3]
        ymin = detections[0, 0, 0, 4]
        xmax = detections[0, 0, 0, 5]
        ymax = detections[0, 0, 0, 6]

        className = classNames[int(label)]

        cv2.rectangle(color_image, (int(xmin * expected), int(ymin * expected)),
                      (int(xmax * expected), int(ymax * expected)), (255, 255, 255), 2)
        cv2.putText(color_image, className,
                    (int(xmin * expected), int(ymin * expected) - 5),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255))

        # plt.imshow(crop_img)

        scale = height / expected
        xmin_depth = int((xmin * expected + crop_start) * scale)
        ymin_depth = int((ymin * expected) * scale)
        xmax_depth = int((xmax * expected + crop_start) * scale)
        ymax_depth = int((ymax * expected) * scale)
        xmin_depth, ymin_depth, xmax_depth, ymax_depth
        cv2.rectangle(colorized_depth, (xmin_depth, ymin_depth),(xmax_depth, ymax_depth), (255, 255, 255), 2)

        images = np.hstack((color_image, colorized_depth))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)  ### use images if you want to stack them // use color_image for only image // use colorized_depth for depth information

        depth = np.asanyarray(aligned_depth_frame.get_data())
        # Crop depth data:
        depth = depth[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)

        # Get data scale from the device and convert to meters
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth = depth * depth_scale
        dist, _, _, _ = cv2.mean(depth)
        print("Detected a {0} {1:.3} meters away.".format(className, dist))

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:

    # Stop streaming
    pipeline.stop()