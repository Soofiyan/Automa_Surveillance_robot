import pytumblr

# AuthenticateS via OAuth, copy from https://api.tumblr.com/console/calls/user/info
client = pytumblr.TumblrRestClient(
  'd2s91b3ZryGwNtIiTEBbQiay6YovRv1BPt3wF1ArynR4makCbS',
  '9lIrzDN0DWCMWhKNKiZT7MTw0lB2fOKYJcmbhhm242HavhgFCL',
  'lfYx4QydcY49G3ua76jQRIrfuxu1gQSwm9IuDrPtD5y58FnKTe',
  'Kkl2ogUPMx5gOTff9iQ3Hnx4KhLFy81AgFJ2H7AV0g75ueX1vO'
)

client.create_photo(
'soofiyan',
state="published",
tags=["raspberrypi", "picamera"],
data="/Applications/Codes/Python/input.mp3"
)
print("uploaded")

client.create_audio("soofiyan.tumblr.com", caption="Rock out", data="/Applications/Codes/Python/input.mp3")
# Authenticate via API Key
# client.dashboard()