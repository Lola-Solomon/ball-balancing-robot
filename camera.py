import cv2
import numpy as np

class USBCamera:
    def __init__(self, cam_index=2):
        # Open USB camera using V4L2 (IMPORTANT on Ubuntu)
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("ERROR: Camera not opened")
            exit(1)

        # Camera resolution
        self.width = 640
        self.height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # RED color HSV ranges (two ranges)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

    def take_pic(self):
        # Capture a frame
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def find_ball(self, image):
        # Convert BGR → HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create red masks
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = mask1 | mask2

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Largest contour = ball
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 300:  # ignore noise
                (x, y), radius = cv2.minEnclosingCircle(c)

                # Draw detected ball
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(image, (int(x), int(y)), 4, (0, 0, 255), -1)

                # Convert to centered coordinates
                x_centered = x - self.width / 2
                y_centered = y - self.height / 2

                # Swap axes to match platform coordinates
                #x_centered, y_centered = -y_centered, x_centered

                x_centered = -x_centered

                return int(x_centered), int(y_centered), int(area)

        return -1, -1, 0

    def clean_up_cam(self):
        self.cap.release()
        cv2.destroyAllWindows()


# ================= MAIN LOOP =================
cam = USBCamera()  # <-- USB Rapoo camera

print("Press 'q' to quit")

while True:
    frame = cam.take_pic()
    if frame is None:
        continue

    x, y, area = cam.find_ball(frame)
    if(x==-1 or y==-1):
        print("ball lost")

    # Draw image center
    cx = cam.height // 2
    cy = cam.width // 2
    center=(cy,cx)
    cv2.circle(frame, (cy, cx), 5, (255, 0, 0), -1)
    axis_length=150
    x_cam = (center[0] + axis_length, center[1])

# Y camera axis (down)
    y_cam = (center[0], center[1] + axis_length)

    cv2.arrowedLine(frame, center, x_cam, (0, 0, 255), 2)   # Red
    cv2.arrowedLine(frame, center, y_cam, (0, 255, 0), 2)   # Green

    cv2.putText(frame, "X_cam", (x_cam[0] + 5, x_cam[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

    cv2.putText(frame, "Y_cam", (y_cam[0] + 5, y_cam[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # Show video
    cv2.imshow("Red Ball Tracking", frame)

    # Print ball coordinates
    if area > 0:
        print(f"Ball position (centered): x={x}, y={y}, area={area}")

    # Exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    

cam.clean_up_cam()
