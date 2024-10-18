import cv2
import cv2.aruco as aruco
import numpy as np
import math

# Constants
DICT_USED = aruco.DICT_6X6_50
WINDOW_WIDTH = 1920  # pixels
WINDOW_HEIGHT = 1080
X_DIST = 40  # Real-world X distance in cm
Y_DIST = 50  # Real-world Y distance in cm
MARKER_LENGTH = 5  # Marker length in cm (for pose estimation)

# Load ArUco dictionary and parameters
dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# Camera calibration parameters (for pose estimation)
camera_matrix = np.array([[800, 0, WINDOW_WIDTH / 2],
                          [0, 800, WINDOW_HEIGHT / 2],
                          [0, 00, 1]], dtype="double")
dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

# Open the camera
cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)


# Function to calculate Euclidean distance between two points
def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


# Function to normalize a vector
def unit_vector(vector):
    return vector / np.linalg.norm(vector)


# Function to calculate the angle of the marker based on its corners
def getAngle(mc):
    x1, y1 = mc[0][0][0], mc[0][0][1]
    x2, y2 = mc[0][1][0], mc[0][1][1]
    x3, y3 = mc[0][2][0], mc[0][2][1]
    x4, y4 = mc[0][3][0], mc[0][3][1]

    mx1, my1 = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    mx2, my2 = (x3 + x4) / 2.0, (y3 + y4) / 2.0
    mX, mY = (x3 + x4) / 2.0, (y3 + y4) / 2.0
    centerX = (mx1 + mx2) / 2.0
    centerY = (my1 + my2) / 2.0

    v1 = unit_vector([centerX - mX, centerY - mY])
    v2 = unit_vector([-1, 0])  # Reference vector along the X-axis
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    if mY - centerY > 0:
        return 2 * math.pi - angle

    return angle


# Initialize variables
scaling_factor_x = None
scaling_factor_y = None
foundCorners = False

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

        # Find reference markers (ID 0 and ID 3)
        index0, index3 = -1, -1
        for i in range(len(markerIDs)):
            if markerIDs[i] == 0:
                index0 = i
            elif markerIDs[i] == 3:
                index3 = i

        # If reference markers are found, calculate scaling factors
        if index0 != -1 and index3 != -1 and not foundCorners:
            m0x = markerCorners[index0][0][0][0]
            m0y = markerCorners[index0][0][0][1]
            m3x = markerCorners[index3][0][0][0]
            m3y = markerCorners[index3][0][0][1]
            print(f": {m0x} {m0y} cm/pixel, Y: {m3x} {m3y} cm/pixel")

            # Calculate scaling factors based on real-world distances
            scaling_factor_x = X_DIST / (m3x - m0x)
            scaling_factor_y = Y_DIST / (m3y - m0y)
            foundCorners = True
            print(f"Scaling Factors - X: {scaling_factor_x} cm/pixel, Y: {scaling_factor_y} cm/pixel")

        # Process other markers (the agent, for example)
        for i in range(len(markerIDs)):
            if markerIDs[i] not in [0, 3]:  # Exclude reference markers
                mc = markerCorners[i]

                # Get the center of the third marker (agent)
                centerX = np.mean(mc[0][:, 0])
                centerY = np.mean(mc[0][:, 1])

                # Convert pixel coordinates to real-world coordinates
                real_world_x = (centerX - m0x) * scaling_factor_x
                real_world_y = (centerY - m0y) * scaling_factor_y
                print(f"Scaling Factors - X: {scaling_factor_x} cm/pixel, Y: {scaling_factor_y} cm/pixel")

                # Get the angle of the marker
                angle = getAngle(mc)
                angle_degrees = np.degrees(angle)

                print(
                    f"Marker ID: {markerIDs[i][0]}, Real-World Position - X: {real_world_x:.2f} cm, Y: {real_world_y:.2f} cm")
                print(f"Orientation Angle: {angle_degrees:.2f}°")

    # Display the frame with detected markers
    cv2.imshow('ArUco Marker Detection', frame)

    # Press 'q' to quit the video capture
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
