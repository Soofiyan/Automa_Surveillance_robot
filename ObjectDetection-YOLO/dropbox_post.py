# import dropbox

# class TransferData:
#     def __init__(self, access_token):
#         self.access_token = access_token

#     def upload_file(self, file_from, file_to):
#         """upload a file to Dropbox using API v2
#         """
#         dbx = dropbox.Dropbox(self.access_token)

#         with open(file_from, 'rb') as f:
#             dbx.files_upload(f.read(), file_to)

# def main():
#     access_token = '-9R9oORmWtAAAAAAAAAAFAkIJUtqc2lAOFNgL6ff06e5lW2pgaM124o_P5ih4bKo'
#     transferData = TransferData(access_token)

#     file_from = 'git.pdf'
#     file_to = '/test_dropbox/git.pdf'  # The full path to upload the file to, including the file name

#     # API v2
#     transferData.upload_file(file_from, file_to)

# if __name__ == '__main__':
#     main()

# import dropbox
# access_token = '-9R9oORmWtAAAAAAAAAAFAkIJUtqc2lAOFNgL6ff06e5lW2pgaM124o_P5ih4bKo'
# file_from = 'yolov3.weights'  #local file path
# file_to = '/yolov3.weights'      # dropbox path
# def upload_file(file_from, file_to):
#     dbx = dropbox.Dropbox(access_token)
#     f = open(file_from, 'rb')
#     dbx.files_upload(f.read(), file_to)
# upload_file(file_from,file_to)
import dropbox
from datetime import datetime
import os
# datetime object containing current date and time

now = datetime.now()
# dd/mm/YY H:M:S
dt_string = now.strftime("%Y/%m/%d_%H:%M:%S")
# print("date and time =", dt_string)



file_from = 'git.pdf' #local file path
file_to = '/%s.pdf'%dt_string     # dropbox path
file_path = file_from
dest_path = file_to
access_token = '-9R9oORmWtAAAAAAAAAAIYq1GMe6lWwmUZyA5GV4iyRc8p3bU9ppKR9qXddrqand'
f = open(file_path,'rb')
file_size = os.path.getsize(file_path)
dbx = dropbox.Dropbox(access_token)
CHUNK_SIZE = 10 * 1024 *1024

if file_size <= CHUNK_SIZE:
    dbx = dropbox.Dropbox(access_token)
    f = open(file_from, 'rb')
    dbx.files_upload(f.read(), file_to)

else:

    upload_session_start_result = dbx.files_upload_session_start(f.read(CHUNK_SIZE))
    cursor = dropbox.files.UploadSessionCursor(session_id=upload_session_start_result.session_id,
                                               offset=f.tell())
    commit = dropbox.files.CommitInfo(path=dest_path)

    while f.tell() < file_size:
        if ((file_size - f.tell()) <= CHUNK_SIZE):
            print(dbx.files_upload_session_finish(f.read(CHUNK_SIZE),
                                            cursor,
                                            commit))
        else:
            dbx.files_upload_session_append(f.read(CHUNK_SIZE),
                                            cursor.session_id,
                                            cursor.offset)
            cursor.offset = f.tell()