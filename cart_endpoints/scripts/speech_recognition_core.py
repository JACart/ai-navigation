#!/usr/bin/env python

import rospy
import pyaudio  
import wave 
import speech_recognition as sr
import sys
import os
import time
from playsound import playsound
import vlc
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix


class speech_recognition_core(object):

    def __init__(self):
        self.active = 0
        self.text_passing = False
        self.ping_in_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/ai-navigation/cart_endpoints/sounds/', 'ping_in.mp3'))
        self.ping_out_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/ai-navigation/cart_endpoints/sounds/', 'ping_out.mp3'))
        self.emergency_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/ai-navigation/cart_endpoints/sounds/', 'Emergency.mp3'))
        
        rospy.init_node('speech_recognition')
        rospy.loginfo("Starting Speech Recognition Node!")
        self.pullover_pub = rospy.Publisher('/pullover', Bool, queue_size=10)
        self.speech_text_pub = rospy.Publisher('/speech_text', String, queue_size=10)
        self.location_speech_sub = rospy.Subscriber('/location_speech', Bool, self.location_speech_callback)
        
        #This loops helps to identify all avaliable microphones

        self.m = sr.Microphone() #can set the microphone index here
        self.r = sr.Recognizer()
        
        #experimental ambient noise adjustment
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
            
        self.r.listen_in_background(self.m, self.listener, phrase_time_limit=6)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #self.loop()
            rate.sleep()

    def listener(self, recognizer, audio):
        try:
            #audio = self.r.listen(source, phrase_time_limit=5)
            text = recognizer.recognize_google(audio)
            text = text.lower()
            text_array = text.split()
            if self.text_passing:
                self.speech_text_pub.publish(text)
            print(text)
            #Alucard, autocorrect, autocart possible other words that need added
            for x in range(len(text_array)):
                #checks for single words like autocart and skips looking for individual words if it is found
                if self.active <= 0:
                    if text_array[x] == "alucard" or text_array[x] == "autocorrect" or text_array[x] == "autocart" or text_array[x] == "autocar":
                        self.ping_in_sound.stop()
                        self.ping_in_sound.play()
                        self.active = 2
                #checks for two words that together form autocart
                if self.active > 0 or text_array[x] == "auto":
                    if (self.active > 0 or len(text_array) > x+1 and (text_array[x+1] == "cart" or text_array[x+1] == "part" 
                        or text_array[x+1] == "parts" or text_array[x+1] == "carton" or text_array[x+1] == "kurt" or text_array[x+1] == "card")):
                        #The user can make a full request in one go or two goes, (ie "Auto cart help" or "Auto cart...'ping in'... help")
                        #self.active basically allows the speech to be recongized for one loop after saying auto cart to support this design
                        print("Processed: " + text)
                        for y in range(x, len(text_array)):
                            if text_array[y] == "indicated":
                                #this is a temporary fix to the cart hearing the emergency message playing
                                break
                            if text_array[y] == "terminate":
                                print("Termination")
                                #end the ride... do we still want/need this functionality?
                                break
                            if text_array[y] == "help" or text_array[y] == "stop" or text_array[y] == "emergency":
                                self.emergency_sound.stop()
                                self.emergency_sound.play()
                                time.sleep(4)
                                print("Emergency Issued")
                                self.active = 3
                                self.pullover_pub.publish(True)
                                break
                            if text_array[y] == "cancel" or text_array[y] == "resume":
                                print("Emergency Canceled")
                                self.active = 1
                                self.pullover_pub.publish(False)
                                break
                            
                        if self.active > 0:
                            self.active -= 1
                            if self.active <= 0:
                                self.ping_out_sound.stop()
                                self.ping_out_sound.play()
                        else:
                            #stop is done first as the previous play must be ended to play again
                            self.ping_in_sound.stop()
                            self.ping_in_sound.play()
                            self.active = 1
                        break

        except Exception as e:
            if self.active > 0:
                self.active -= 1
                if self.active <= 0:
                    self.ping_out_sound.stop()
                    self.ping_out_sound.play()
            print("Error, typically means there was no registered english speech: " + str(e))
         
         
    def location_speech_callback(self, msg):
        print(str(msg.data))
        if msg.data:
            #send text to server
            print("start the text passing")
            self.text_passing = True
        else:
            self.text_passing = False
            #stop sending text to server
            
        
if __name__ == "__main__":
    try:
        speech_recognition_core()
    except rospy.ROSInterruptException:
        pass