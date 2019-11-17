#!/usr/bin/env python

#Autonomous Speech Recognition
#By Jacob Hitzges

import pyaudio  
import wave 
import speech_recognition as sr
import sys
import os.path
import time
from playsound import playsound
import simpleaudio as sa

CHUNK_SIZE = 1024
flag = False
notAtDestination = True
waiting = True


def loop():
    r = sr.Recognizer()

    with sr.Microphone() as source:


        try:
        
            audio = r.listen(source, timeout=3, phrase_time_limit=3)

        except:
            pass


        notAtDestination = True

        #if (parsedData['command'] == 'arrived'):
        #    endRide()

        while notAtDestination == True:

            try:
                flag = False
                print("say something1")
                audio = r.listen(source, timeout=4, phrase_time_limit=4)


        
                text = r.recognize_google(audio)

                print("you said " + text)

                if text == "Auto cart" or text == "Auto Parts" or text == "Auto Part" or text == 'Alucard' or text == 'Olive Garden' or text == "Auto carton" or text == 'Albuquerque' or text == 'Elkhart' or text == "autocorrect":

                    playsound('ping in.mp3')
                                   

                    try:

                        print("Say something2")
                        audio = r.listen(source, timeout=4, phrase_time_limit=4)
                        text = r.recognize_google(audio)
                        
                        text = text.split()

                        for x in range(len(text)):

                            if text[x] == 'terminate':

                                print("terminated")
                                endRide()
                                notAtDestination = False
                                

                            if text[x] == 'help' or text[x] == 'emergency' or text[x] == 'stroke' or text[x] == 'attack' or text[x] == 'stop':


                                 playsound('Emergency.mp3')
                                
                                 
                                 try:

                                     print("say cancel if you want to cancel")
                                     audioCancel = r.listen(source, timeout=4, phrase_time_limit=4)
                                     textCancel = r.recognize_google(audioCancel)

                                     print(textCancel)

                                     textCancel = textCancel.split()

                                     print(textCancel)

                                     for x in range(len(textCancel)):

                                         if textCancel[x] == 'cancel':
                                             print("canceled")
                                             flag = True
                                         else:
                                             pass
                                             sendStop()


                                
                                       
                                     

                                     if (flag == False):
                                         print("Emergency")  
                                    
                                     playsound('ping out.mp3')
                                     

                                 except:
                                     playsound('ping out.mp3')


                            if text[x] == 'stop':


                                print("Emergency! We are stopping the cart")
                                
                            else:
                                pass

                   

                    except:
                        print ("Translation failed." + text)
                        playsound('ping out.mp3')


                
                                
                else:
                    
                    pass


            except:
                pass
                


loop()

def beginRide():

    inCart = False
    inCart = True

    if inCart == True:
        playsound('enter.mp3')
    else:
        pass
                                
        

def endRide():
    atDestination = False
    atDestination = True
    notAtDestination = False

    if atDestination == True:
        playsound('exit.mp3')
    else:
        pass

def sendStop():
    data = {
        "command": "passenger_stop",
        "data":"passenger_stop"
    

    }
    client.publish("/audio",json.dumps(data))

def onMessage(data):
    parsedData = json.loads(data)
    if(parsedData['command'] == 'transitawait'):
        playsound('enter.mp3')
        
        r = sr.Recognizer()

        with sr.Microphone() as source:
  

            try:
        
                audio = r.listen(source, timeout=3, phrase_time_limit=3)

                textDestination = r.recognize_google(audio)

                textDestination = textDestination.split()

                for x in range(len(textDestination)):

                    if textDestination[x] == 'Cafeteria':

                        self.sendDestC()

                for x in range(len(textDestination)):

                    if textDestination[x] == 'Entrance':

                        sendDestE()
                                
                for x in range(len(textDestination)):

                    if textDestination[x] == 'Clinic':

                        sendDestCl
                                
                for x in range(len(textDestination)):

                    if textDestination[x] == 'Garage':

                        sendDestG()

                for x in range(len(textDestination)):

                    if textDestination[x] == 'labs':

                        sendDestL()

                

            except:
                pass        


        
        loop()

        if (parsedData['command'] == 'arrived'):
            endRide()
        

        
        elif (parsedData['command'] == 'arrived'):
            endRide()


      


def sendDestC():

    data = {
        "command": "passenger_Cafeteria_Speech",
        "data": "Cafeteria"
    }

def sendDestE():

    data = {
        "command": "passenger_Entrance_Speech",
        "data": "Entrance"
    }

def sendDestCl():

    data = {
        "command": "passenger_Clinic_Speech",
        "data": "Clinic"
    }

def sendDestG():

    data = {
        "command": "passenger_garage_Speech",
        "data": "garage"
    }

def sendDestL():

    data = {
        "command": "passenger_labs_Speech",
        "data": "labs"
    }


        
