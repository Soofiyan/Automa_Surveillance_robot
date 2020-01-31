# /*
# *
# * Project Name: 	sms send on alert
# * Author List: 	Soofiyan Atar
# * Filename: 		send_sms.py
# * Functions: 		None
# * Global Variables:	None
# *
# */

import requests
 
url = "https://www.fast2sms.com/dev/bulk"

print("Enter your number    ")
number = input()
 
payload = "sender_id=FSTSMS&message=test&language=english&route=p&numbers={}".format(number)
headers = {
 'authorization': "BEwV21NpDcinrksYqIa73K5uLzb6yjexCZogSOGWlUt9FRJAdM3tUEuNFSKwfd2gX4veyCjpMhaImYHi",
 'Content-Type': "application/x-www-form-urlencoded",
 'Cache-Control': "no-cache",
 }
 
response = requests.request("POST", url, data=payload, headers=headers)
 
print(response.text)