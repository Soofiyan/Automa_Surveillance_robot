# /*
# *
# * Project Name: 	Chatbot for basic commands to navigate bot
# * Author List: 	Soofiyan Atar
# * Filename: 		speech.py
# * Functions: 		None
# * Global Variables:	None
# *
# */
import speech_recognition as sr
import ctypes
import difflib
def callback(recognizer, audio):                    # this is called from the background thread
    global audio_string
    audio_string = ''
    try:
        audio_string = recognizer.recognize_google(audio)
        # print("You said " + recognizer.recognize_google(audio))  # received audio data, now need to recognize it
    except Exception as e:
        e = 0
        # print("Oops! Didn't catch that")
r = sr.Recognizer()
r.listen_in_background(sr.Microphone(device_index=0),callback,phrase_time_limit=5)
with sr.Microphone(device_index=0) as source:
        print(sr.Microphone.list_microphone_names())

import time
Dict = { 'where are you':'1','kidhar ho':'1','hero':'1' ,
'landmark':'2', 'nishan':'2', 'neeshan':'2','nishant':'2', 'vishal':'2','ishan':'2',
'charge':'3','power on':'3','plug':'3','flag':'3','club':'3','clerk':'3','log':'3','lag':'3',
'uncharge':'4','power off':'4', 'power of':'4','unplug':'4','an charge':'4','can charge':'4','incharge':'4','recharge':'4','discharge':'4',
'forward':'5', 'aage':'5',
'stay here':'6','idhar rukho':'6', 'idhar ruko':'6','kidhar ko':'6', 'idhar uco':'6', 'idhar dukkho':'6','stair' :'6','stay yaar':'6','steer':'6','dear':'6',
'surveillance':'7', 'nigrani':'7',
'backward':'8', 'piche':'8',
'left':'9', 'bye':'9', 'baye':'9','blessed':'9','flash':'9','best':'9','last':'9',
'right':'10', 'daye':'10' , 'die':'10','flight':'10','write':'10','flight':'10','tight':'10'
}
audio_string = ''
biggest_size = 2
closest_audio = ''

while True:
    if(audio_string != ''):
        audio_string = audio_string.casefold()
        print(audio_string)
        try:
            output_sound = int(Dict[audio_string])
            print(Dict[audio_string])
        except Exception as e:
            # print("not in dictionary")
            for similar_audio in Dict:
                matches = difflib.SequenceMatcher(None, similar_audio, audio_string)
                similar_audio = str(similar_audio)
                audio_string = str(audio_string)
                match = matches.find_longest_match(0, len(similar_audio), 0, len(audio_string))
                if biggest_size < match.size:
                    biggest_size = match.size
                    closest_audio = similar_audio
            if closest_audio == '':
                output_sound = 20
            else:
                output_sound = int(Dict[closest_audio])
                print(output_sound)
                closest_audio = ''
                biggest_size = 2
            e = 0
        audio_string = ''
    time.sleep(0.1)
