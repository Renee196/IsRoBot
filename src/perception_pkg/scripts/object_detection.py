#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize ROS node
rospy.init_node('object_detector_node')

# Load YOLO
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")  # You can add the path

# Loading coco.names file, which contains names of objects it can detect
with open("coco.names", 'r') as f:
    classes = [line.strip() for line in f]

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

bridge = CvBridge()
image_data = None

def image_callback(msg):
    global image_data
    image_data = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

# ROS Subscriber for simulated camera image
rospy.Subscriber('/camera/image', Image, image_callback)

# ROS Publisher for object detection results
object_detection_pub = rospy.Publisher('object_detection', String, queue_size=10)

# Looping creates N_images to look like video
while not rospy.is_shutdown():
    if image_data is not None:
        frame = image_data.copy()

        class_ids = []
        confidences = []

        height, width = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        net.setInput(blob)
        outputs = net.forward(output_layers)

        for out in outputs:
            for i in out:
                scores = i[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.6:
                    class_ids.append(class_id)
                    confidences.append(float(confidence))

        for i in range(len(class_ids)):
            conf = confidences[i]
            label = classes[class_ids[i]]
            print(label, conf * 100)

            voice = str(label) + " in front of you"

        file_path = 'voice.mp3'
        sound = gTTS(text=voice, lang='en')
        sound.save(file_path)
        if class_ids:
            playsound(file_path)
            # Publish object detection result to ROS topic
            object_detection_pub.publish(String(label))

    rospy.sleep(0.1)  # Adjust the sleep time based on the simulation time step
