from openai import OpenAI
import os 
import base64


api_key = os.getenv("GEMINI_API_KEY")

client = OpenAI(
    api_key=api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)


# Function to encode the image
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')



def llm_response(image_path):
    # Getting the base64 string
    base64_image = encode_image(image_path)


    response = client.chat.completions.create(
        model="gemini-2.0-flash",
        n=1,
        messages=[
            {
                "role": "system", 
                "content": 
                '''You are a image captioner, you will be given yolo detected images and you should 
                    1. caption the images based on what you see in the image '''
            },
            {
                "role": "user",
                # "content": "Explain to me how AI works"
                "content": [
                    {
                        "type": "text",
                        "text": "What is in this image?",
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url":  f"data:image/jpeg;base64,{base64_image}"},
                    }
                ]
            }
        ]
    )

    return response.choices[0].message.content


response = llm_response("/home/dock/turtlebot_ws/src/detection1.png")

print(response)

# save as json
import json
import datetime

data={
   "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
   "description": response
}

with open("response.txt", "a") as file:
    json.dump(data, file, indent=4)
    file.write("\n")