# import nexmo

# client = nexmo.Client(key='094fa742', secret='b5dvOcN0PASGPw7e')

# client.send_message({
#     'from': 'Nexmo',
#     'to': '917506183099',
#     'text': 'Hello from Nexmo',
# })

import requests
 
url = "https://www.fast2sms.com/dev/bulk"
 
payload = "sender_id=FSTSMS&message=test&language=english&route=p&numbers=7506183099"
headers = {
 'authorization': "BEwV21NpDcinrksYqIa73K5uLzb6yjexCZogSOGWlUt9FRJAdM3tUEuNFSKwfd2gX4veyCjpMhaImYHi",
 'Content-Type': "application/x-www-form-urlencoded",
 'Cache-Control': "no-cache",
 }
 
response = requests.request("POST", url, data=payload, headers=headers)
 
print(response.text)