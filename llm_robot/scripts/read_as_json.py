# import json

# # Open the text file and load it as JSON
# with open('/home/dock/turtlebot_ws/response.txt', 'r') as file:
#     data = json.load(file)

# # Print the loaded data
# print(data)


import pandas as pd

data = {
    'Name': ['Alice', 'Bob', 'Charlie'],
    'Age': [25, 30, 28],
    'City': ['New York', 'London', 'Paris']
}

df = pd.DataFrame(data)
filename = "output.csv"
df.to_csv(filename, index=False)
