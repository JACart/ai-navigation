
import pyaudio  
import speech_recognition as sr


recognize = False

def startRecognize(callback):
    recognize = True
    recognizeHelper(callback)

def recognizeHelper(callback):
    r = sr.Recognizer()
    while(recognize):
        with sr.Microphone() as source:
            try:
                audio = r.listen(source, timeout=3, phrase_time_limit=3)
            except:
                pass
            try:
                text = r.recognize_google(audio)
                print(text)
                callback(text)
            except:
                pass

def stopRecognize():
    recognize = False

