# import speech_recognition as sr 
  
# #enter the name of usb microphone that you found 
# #using lsusb 
# #the following name is only used as an example 
# mic_name = "USB Device 0x46d:0x825: Audio (hw:1, 0)"
# #Sample rate is how often values are recorded 
# sample_rate = 48000
# #Chunk is like a buffer. It stores 2048 samples (bytes of data) 
# #here.  
# #it is advisable to use powers of 2 such as 1024 or 2048 
# chunk_size = 2048
# #Initialize the recognizer 
# recording = sr.Recognizer() 
  
# #generate a list of all audio cards/microphones 
# mic_list = sr.Microphone.list_microphone_names() 
  
# #the following loop aims to set the device ID of the mic that 
# #we specifically want to use to avoid ambiguity. 
# # for i, microphone_name in enumerate(mic_list): 
# #     if microphone_name == mic_name: 
# #         device_id = i 
# device_id = 0

# while True:
#     #use the microphone as source for input. Here, we also specify  
#     #which device ID to specifically look for incase the microphone  
#     #is not working, an error will pop up saying "device_id undefined" 
#     with sr.Microphone(device_index = device_id) as source: 
#         recording.adjust_for_ambient_noise(source)
#         print("Please Say something:")
#         audio = recording.listen(source)
#         try:
#             print("You said: \n" + recording.recognize(audio))
#             if(recording.recognize(audio) == "exit"):
#                 print("now close the chatbot")
#         except Exception as e:
#             print(e)

# import speech_recognition as sr
# r = sr.Recognizer()
# with sr.Microphone() as source:                # use the default microphone as the audio source
#     audio = r.listen(source)                   # listen for the first phrase and extract it into audio data

# try:
#     print("You said " + r.recognize(audio))    # recognize speech using Google Speech Recognition
# except LookupError:                            # speech is unintelligible
#     print("Could not understand audio")
    
    
# #Example 2

# import speech_recognition as sr
# r = sr.Recognizer()
# with sr.WavFile("test.wav") as source:              # use "test.wav" as the audio source
#     audio = r.record(source)                        # extract audio data from the file

# try:
#     print("Transcription: " + r.recognize(audio))   # recognize speech using Google Speech Recognition
# except LookupError:                                 # speech is unintelligible
#     print("Could not understand audio")
    
    
# #Example 3

# #Transcribe a WAV audio file and show the confidence of each:

# import speech_recognition as sr
# r = sr.Recognizer()
# with sr.WavFile("test.wav") as source:              # use "test.wav" as the audio source
#     audio = r.record(source)                        # extract audio data from the file

# try:
#     list = r.recognize(audio,True)                  # generate a list of possible transcriptions
#     print("Possible transcriptions:")
#     for prediction in list:
#         print(" " + prediction["text"] + " (" + str(prediction["confidence"]*100) + "%)")
# except LookupError:                                 # speech is unintelligible
#     print("Could not understand audio")
    
    
#Example 4

#Listening in the background:

import speech_recognition as sr
def callback(recognizer, audio):                          # this is called from the background thread
    try:
        print("You said " + recognizer.recognize_google(audio))  # received audio data, now need to recognize it
    except Exception as e:
        print(e)
r = sr.Recognizer()
r.listen_in_background(sr.Microphone(device_index=0), callback)

import time
while True: 
    time.sleep(0.1) 