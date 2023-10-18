import os

files = os.listdir('.')

for file in files:
    if file.endswith('.obj'):
        os.remove(file)
