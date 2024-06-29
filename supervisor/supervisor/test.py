import json

# Define the path to the JSON file
file_path = '/home/yomnahashem/Formula24/src/fs-system/supervisor/supervisor/test.json'

# Open the JSON file and load its data
with open(file_path, 'r') as file:
    data = json.load(file)

# Accessing data from the JSON
print(data)
print("Name:", data['name'])
print("Age:", data['age'])
print("City:", data['city'])
print("Skills:", ', '.join(data['skills']))
