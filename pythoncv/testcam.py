import cv2
import cv2.aruco as aruco

print(cv2.cuda.getCudaEnabledDeviceCount())
print(f"{cv2.CAP_PROP_FRAME_WIDTH} x {cv2.CAP_PROP_FRAME_HEIGHT}")

# Constants
DICT_USED = aruco.DICT_6X6_50
WINDOW_WIDTH = 1400  # pixels
WINDOW_HEIGHT = 1100

# Load ArUco dictionary and parameters
dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# Open the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # add this line

while True:
    # Capture frame-by-frame
    success, frame = cap.read()

    if not success:
        print("Error: Unable to capture video.")
        break

    # Detect ArUco markers in the frame
    markerCorners, markerIDs, _ = detector.detectMarkers(frame)

    if markerIDs is not None:
        frame = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)

    # Display the frame with detected markers
    cv2.imshow('ArUco Marker Detection', frame)

    # Press 'q' to quit the video capture
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
