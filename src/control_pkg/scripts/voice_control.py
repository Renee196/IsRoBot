import speech_recognition as sr
import rospy

class VoiceControl():
    def __init__(self) -> None:
        pass

    

    def voice_recognition():
        r = sr.Recognizer()
        mic= sr.Microphone()
        with mic as source:
            r.adjust_for_ambient_noise(source)
            audio = r.listen(source)

        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        try:
            response["transcription"] = r.recognize_google(audio)
        except sr.RequestError:
            # API was unreachable or unresponsive
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            # speech was unintelligible
            response["error"] = "Unable to recognize speech"

        return response
    
    def voice_control():
        if 'forward' in response:
        elif 'backward' or 'back' in response:
        elif 'left' in response:
        elif 'right' in response:
        elif 'isrobot' and 'autonomous' in response:
        elif 'isrobot stop' or 'isrobot wait' in response:
        elif 'isrobot' and 'voice assistant' in response:
        

if __name__=='__main__':
    
    recognise=

























