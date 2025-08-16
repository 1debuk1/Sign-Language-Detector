from flask import Flask, request, jsonify
import cv2
import numpy as np
import tensorflow as tf

app = Flask(__name__)

# Load your trained ML model 
model_path = "sign_language_model.h5"
model = tf.keras.models.load_model(model_path)

# Load labels from file
with open('output_labels.txt', 'r') as f:
    labels = [line.strip() for line in f.readlines()]

@app.route('/detect', methods=['POST'])
def detect_sign_language():
    if request.method == 'POST':
        print("Receiving image...")

        # Read image from request
        nparr = np.frombuffer(request.data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Preprocess the image
        image_resized = cv2.resize(image, (224, 224))  # Assuming 224x224 input size
        input_data = np.expand_dims(image_resized, axis=0).astype(np.float32)
        input_data = input_data / 255.0  # Normalize to [0,1]

        # Run inference
        predictions = model.predict(input_data)
        detected_sign_index = np.argmax(predictions[0])
        confidence = np.max(predictions[0])

        # Map index to sign language label
        detected_sign = labels[detected_sign_index]

        # Check confidence level
        if confidence >= 0.8:
            response = jsonify({'detected_sign': detected_sign, 'confidence': confidence})
            print(f"Detected: {detected_sign} with confidence: {confidence}")
        else:
            response = jsonify({'detected_sign': '', 'confidence': 0.0})
            print(f"Low confidence detection")

        return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

