#!/usr/bin/env python

import rospy
import numpy as np
from gtts import gTTS
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from custom_msgs.srv import PauseResume, PauseResumeResponse

class QRScanner:
    def __init__(self):
        self.pause_service = rospy.Service('pause_resume', PauseResume, self.handle_pause_resume)
        self.bridge = CvBridge()
        self.qr_subscriber = rospy.Subscriber('isrobot/side_cam/image_raw', Image, self.image_callback)
        self.pause = False
        self.succeeded = False
        self.sound_client = SoundClient()

    def image_callback(self, msg):
        if not self.pause:
            # Process the image and extract QR code information
            qr_code_info = self.detect_qr_code(msg)

            if qr_code_info:
                # Perform text-to-speech based on QR code information
                self.text_to_speech(qr_code_info)
                self.succeeded = True

    def detect_qr_code(self, image_msg):
        try:
            # Convert the ROS Image message to a NumPy array
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

            # Convert the image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Use OpenCV QR code detector
            qr_detector = cv2.QRCodeDetector()
            retval, decoded_info, points = qr_detector.detectAndDecodeMulti(gray)

            if retval:
                # Detected QR code(s)
                qr_info = ""
                for info in decoded_info:
                    qr_info += f"{info}\n"

                return qr_info.strip()

        except Exception as e:
            rospy.logerr(f"Error detecting QR code: {str(e)}")

        return None

    def text_to_speech(self, text):
        # Use gTTS to convert text to speech
        tts = gTTS(text)
        rospy.sleep(1)
        tts = gTTS(text=text, lang='en')
        tts.save('/tmp/tts.mp3')
        rospy.sleep(5)

    def handle_pause_resume(self, req):
        self.pause = req.pause
        response = PauseResume()
        response.success = self.succeeded
        return response

if __name__ == '__main__':
    try:
        rospy.init_node('scan_qr_code_node')
        qr_scanner = QRScanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

