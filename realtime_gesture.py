import cv2
import numpy as np
from tensorflow import keras

# Paths to model and labels (make sure these files are in the same folder as this script)
model_path = "sign_language_model.h5"
labels_path = "output_labels.txt"

# Load model and labels
model = keras.models.load_model(model_path)
with open(labels_path, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

def preprocess(image):
    image = cv2.resize(image, (224, 224))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.astype(np.float32) / 255.0
    image = np.expand_dims(image, axis=0)
    return image

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

print("Press 'q' to quit")
print("Show your hand in the blue rectangle")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    frame = cv2.flip(frame, 1)
    h, w = frame.shape[:2]
    x1, y1 = w // 4, h // 4
    x2, y2 = 3 * w // 4, 3 * h // 4
    roi = frame[y1:y2, x1:x2]

    if roi.size > 0:
        processed = preprocess(roi)
        preds = model.predict(processed, verbose=0)
        idx = np.argmax(preds[0])
        label = labels[idx]
        conf = preds[0][idx]
        text = f"{label.upper()} ({conf:.2f})"
        print(text)  # Print result to terminal
        cv2.putText(frame, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(frame, "Show hand in blue box", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    cv2.imshow("Sign Language Recognition", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()