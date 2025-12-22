import serial
import time

class ArduinoServo:
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.3)

    def trq_set(self, servo_id, status):
        cmd = f"T,{servo_id},{status}\n"
        self.ser.write(cmd.encode())

    def control_time_rotate(self, servo_id, angle, t):
        time_ms = int(t * 1000)
        cmd = f"M,{servo_id},{angle},{time_ms}\n"
        self.ser.write(cmd.encode())

    # ===== SYNC MOVE =====
    def control_time_rotate_sync(self, angles, t):
        """
        angles = [angle1, angle2, angle3]
        """
        time_ms = int(t * 1000)
        cmd = f"S,{angles[0]},{angles[1]},{angles[2]},{time_ms}\n"
        self.ser.write(cmd.encode())
        print(angles)

    def close(self):
        self.ser.close()


# test_servo = ArduinoServo()
# # test_servo.trq_set(1,1)
# # test_servo.control_time_rotate(1,0,0.01)

# # test_servo.trq_set(2,1)
# # test_servo.control_time_rotate(2,0,0.01)

# # test_servo.trq_set(3,1)
# # test_servo.control_time_rotate(3,0-15,0.01)

# # test_servo.trq_set(1,1)
# # test_servo.trq_set(2,1)
# # test_servo.trq_set(3,1)

# sync_angles = [
#     int(90 + 3),
#     int(90 - 3.5),
#     int(90 - 8)
# ]

# test_servo.control_time_rotate_sync(sync_angles, 1)

# test_servo = ArduinoServo()

# # time.sleep(1.5)  # مهم بعد فتح الـ serial

# # Torque ON
# test_servo.trq_set(1, 1)
# test_servo.trq_set(2, 1)
# test_servo.trq_set(3, 1)

# time.sleep(0.3)

# test_angle = 30

# # IMPORTANT: int angles only
# # sync_angles = [test_angle+14, test_angle+11, test_angle+1]
# sync_angles = [test_angle+14, 0, 0]
# test_servo.control_time_rotate_sync(sync_angles, 1.0)
# sync_angles = [0, 0, 0]
# test_servo.control_time_rotate_sync(sync_angles, 1.0)
# sync_angles = [test_angle+14, 0, 0]
# test_servo.control_time_rotate_sync(sync_angles, 1.0)
# sync_angles = [0, 0, 0]

# test_servo.control_time_rotate_sync(sync_angles, 1.0)


