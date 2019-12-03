#!/usr/bin/env python



import pyaudio  
import wave 
import speech_recognition as sr
import sys
import os.path
import time
from playsound import playsound
#import simpleaudio as sa
import vlc
from ros_client import sendPullOver

CHUNK_SIZE = 1024
flag = False
notAtDestination = True
waiting = True
started = 0

ping_in_sound = vlc.MediaPlayer("ping in.mp3")
ping_out_sound = vlc.MediaPlayer("ping out.mp3")
emergency_sound = vlc.MediaPlayer("Emergency.mp3")

def loop():
    started = 0
    text = "Null"
    r = sr.Recognizer()
    with sr.Microphone() as source:
        notAtDestination = True
        while notAtDestination == True:
            if started <= 0:
                try:
                    flag = False
                    print("say something1")
                    audio = r.listen(source, timeout=10, phrase_time_limit=4)
                    print("testing")
                    text = r.recognize_google(audio)

                    print("you said " + text)

                    if text == "Auto cart" or text == "Auto Parts" or text == "Auto Part" or text == 'Alucard' or text == 'Olive Garden' or text == "Auto carton" or text == 'Albuquerque' or text == 'Elkhart' or text == "autocorrect":
                        ping_in_sound.play()
                        started = 5                
                    else:
                        pass
                    print("test")
                    pass
                except Exception as e:
                    print (e)
                    ping_out_sound.play()
                    pass
            
            if started > 0:
                try:
                    started -= 1
                    print("Say something2")
                    audio = r.listen(source, timeout=10, phrase_time_limit=4)
                    text = r.recognize_google(audio)

                    text = text.split()

                    for x in range(len(text)):

                        if text[x] == 'terminate':
                            started = 0
                            print("terminated")
                            endRide()
                            notAtDestination = False

                        if text[x] == 'help' or text[x] == 'emergency' or text[x] == 'stroke' or text[x] == 'attack' or text[x] == 'stop':
                            emergency_sound.play()
                            started = 0
                            cancel_wait = 1
                            while cancel_wait > 0:
                                try:
                                    cancel_wait -= 1
                                    print("say cancel if you want to cancel")
                                    audioCancel = r.listen(source, timeout=10, phrase_time_limit=4)
                                    textCancel = r.recognize_google(audioCancel)

                                    print(textCancel)

                                    textCancel = textCancel.split()

                                    print(textCancel)

                                    for x in range(len(textCancel)):

                                        if textCancel[x] == 'cancel':
                                            print("canceled")
                                            cancel_wait = 0
                                            flag = True
                                        else:
                                            pass
                                            sendPullOver()
                                    if (flag == False):
                                        print("Emergency")  
                                    ping_out_sound.play()
                                        

                                except:
                                    print ("Translation failed part 3.")
                                    ping_out_sound.play()
                                
                                
                except:
                    print ("Translation failed part 2.")
                    playsound('ping out.mp3')


                    if text[x] == 'stop':
                        print("Emergency! We are stopping the cart") 
                    else:
                        pass



                
                                
                


loop()

def beginRide():

    inCart = False
    inCart = True

    if inCart == True:
        pass#playsound('enter.mp3')
    else:
        pass
                                
        

def endRide():
    atDestination = False
    atDestination = True
    notAtDestination = False

    if atDestination == True:
        pass#playsound('exit.mp3')
    else:
        pass
