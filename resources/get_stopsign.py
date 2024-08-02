import numpy as np
import cv2
import tflite_runtime.interpreter as tflite
import time
import sys

class EdgeTPUDetector:
    def __init__(self, model_path, input_size):
        self.model_path = model_path
        self.input_size = input_size
        self.interpreter = self.load_model()

    def load_model(self):
        self.interpreter = tflite.Interpreter(model_path=self.model_path,experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
        self.interpreter.allocate_tensors()
        return self.interpreter

    def preprocess_image(self, image):
        image = cv2.resize(image, self.input_size)
        image = image.astype(np.float32) / 255.0  # Normalize to [0,1]
        image = np.expand_dims(image, axis=0)  # Add batch dimension
        return image

    def run_inference(self, image):
        input_details = self.interpreter.get_input_details()
        input_data = self.preprocess_image(image)
        
        # Set input tensor
        self.interpreter.set_tensor(input_details[0]['index'], input_data)

        # Run inference
        start_time = time.time()
        self.interpreter.invoke()
        end_time = time.time()
        
        detection_time = end_time - start_time
        detection_time_ms = detection_time * 1000  # Convert seconds to milliseconds
        
        # Get output tensor
        output_details = self.interpreter.get_output_details()
        output_data = self.interpreter.get_tensor(output_details[0]['index'])
        return output_data[0], detection_time_ms  # Remove batch dimension

    def post_process_output(self, output_tensor, conf_threshold=0.5):
        boxes = []
        confidences = []
        class_ids = []

        num_channels = output_tensor.shape[0]
        num_detections = output_tensor.shape[1]
        num_classes = num_channels - 4  # Exclude the 4 box coordinates
        
        for i in range(num_detections):
            box_data = output_tensor[:4, i]
            class_scores = output_tensor[4:, i]

            x_center, y_center, box_width, box_height = box_data
            confidence = np.max(class_scores)
            class_id = np.argmax(class_scores)
            
            #print(f"Detection {i}: Box Data: {box_data}, Class Scores: {class_scores}, Max Confidence: {confidence}")

            if confidence > conf_threshold:
                boxes.append([x_center, y_center, box_width, box_height])
                confidences.append(confidence)
                class_ids.append(class_id)

        return boxes, confidences, class_ids
    
    def get_most_confident_coordinate(self, boxes, confidences, class_ids, target_class_id=3):
        # Filter out detections that do not belong to the target class_id
        filtered_boxes = [box for i, box in enumerate(boxes) if class_ids[i] == target_class_id]
        filtered_confidences = [confidences[i] for i, class_id in enumerate(class_ids) if class_id == target_class_id]

        # Check if there are any detections for the target class_id
        if not filtered_boxes:
            print(f"No detections found for class ID {target_class_id}.")
            return None

        # Find the index of the most confident detection
        max_confidence_index = np.argmax(filtered_confidences)
        most_confident_box = filtered_boxes[max_confidence_index]
        most_confident_confidence = filtered_confidences[max_confidence_index]

        # Convert center coordinates to top-left coordinates for display purposes
        x_center, y_center, box_width, box_height = most_confident_box
        return {
            'x_center': x_center,
            'y_center': y_center,
            'box_width': box_width,
            'box_height': box_height,
            'confidence': most_confident_confidence
        }



    def draw_boxes(self, image, boxes, confidences, class_ids):
        height, width, _ = image.shape

        for i in range(len(boxes)):
            box = boxes[i]
            confidence = confidences[i]
            class_id = class_ids[i]

            x_center, y_center, box_width, box_height = box
            x_center *= width
            y_center *= height
            box_width *= width
            box_height *= height

            x1 = int(x_center - (box_width / 2))
            y1 = int(y_center - (box_height / 2))
            x2 = int(x_center + (box_width / 2))
            y2 = int(y_center + (box_height / 2))

            color = (0, 255, 0)  # Green color for the bounding box
            thickness = 2
            image = cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

            text = f"Class {class_id} Conf {confidence:.2f}"
            image = cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return image
    
    def get_best_coordinate(self, image):
        output_tensor, detection_time_ms = self.run_inference(image)
        print(f'detection time: {detection_time_ms}')
        boxes, confidences, class_ids = self.post_process_output(output_tensor)
        return self.get_most_confident_coordinate(boxes, confidences, class_ids, 3)

if __name__ == "__main__":
    # Define the image path as a string
    image_path = "/Users/nakajimamotoki/Downloads/IMG_4454.JPG"

    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image from path: {image_path}")
        sys.exit(1)

    # Initialize the EdgeTPU detector with the model path
    model_path = "/Users/nakajimamotoki/Downloads/onesixty_integer_quant.tflite"
    input_size = (160, 160)
    detector = EdgeTPUDetector(model_path, input_size)

    # Get the most confident coordinate for the specified class
    result = detector.get_best_coordinate(image)

    if result is not None:
        print(f"Most confident detection for class ID 3:")
        print(f"Center (x, y): ({result['x_center']:.2f}, {result['y_center']:.2f})")
        print(f"Box width: {result['box_width']:.2f}")
        print(f"Box height: {result['box_height']:.2f}")
        print(f"Confidence: {result['confidence']:.2f}")
    else:
        print("No detection found for class ID 3.")

