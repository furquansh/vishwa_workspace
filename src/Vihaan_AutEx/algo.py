from roboflow import Roboflow
import supervision as sv
import cv2

rf = Roboflow(api_key="7ahJ2hFQ6Yh8RhKtwi4X")
project = rf.workspace().project("arrow-detection-mobnet-ssd")
model = project.version(1).model

# Open a connection to the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Perform inference on the frame
    result = model.predict(frame, confidence=40, overlap=30).json()

    labels = [item["class"] for item in result["predictions"]]

    detections = sv.Detections.from_roboflow(result)

    label_annotator = sv.LabelAnnotator()
    bounding_box_annotator = sv.BoxAnnotator()

    annotated_frame = bounding_box_annotator.annotate(
        scene=frame, detections=detections)
    annotated_frame = label_annotator.annotate(
        scene=annotated_frame, detections=detections, labels=labels)

    sv.plot_image(image=annotated_frame, size=(16, 16))

    # Break the loop if 'q' key is pressed
    cv2.waitKey(100) 
        

# Release the webcam and close all windows when done
cap.release()
cv2.destroyAllWindows()

