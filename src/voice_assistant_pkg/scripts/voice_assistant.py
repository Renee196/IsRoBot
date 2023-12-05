#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from gtts import gTTS
import os
import pyttsx3
import speech_recognition as sr
import wolframalpha
import wikipedia

def text_to_speech(text):
    # Using gTTS for text-to-speech
    tts = gTTS(text=text, lang='en', slow=False)
    tts.save('output.mp3')
    os.system('mpg321 output.mp3')  # Assuming you have mpg321 installed

def speech_recognition():
    recognizer = sr.Recognizer()

    with sr.Microphone() as source:
        print("Say Something ")
        recognizer.adjust_for_ambient_noise(source, 2)
        audio = recognizer.listen(source)

        try:
            speech = recognizer.recognize_google(audio)
            return speech
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))

def wolframalpha_query(query):
    # Replace 'Your WolframAlpha App ID here' with your actual WolframAlpha App ID
    app_id = "T3854G-H7KRVG5R4A"
    client = wolframalpha.Client(app_id)

    try:
        res = client.query(query)
        answer = next(res.results).text
        print(answer)
        text_to_speech("Your answer is " + answer)
    except:
        query = query.split(' ')
        query = " ".join(query[0:])
        text_to_speech("I am searching for " + query)
        print(wikipedia.summary(query, sentences=3))
        text_to_speech(wikipedia.summary(query, sentences=3))

def main():
    rospy.init_node('isrobot_node', anonymous=True)

    # Subscribe to the text input topic
    rospy.Subscriber('text_input_topic', String, text_to_speech_callback)

    # Subscribe to the speech input topic
    rospy.Subscriber('speech_input_topic', String, speech_recognition_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

