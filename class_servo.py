import serial
import time

class ArduinoServo:
    def __init__(self, port='/dev/ttyACM1', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Arduino reset

    # Equivalent to trq_set()
    def trq_set(self, servo_id, status):
        """
        status: 1 = torque ON, 0 = torque OFF
        """
        cmd = f"T,{servo_id},{status}\n"
        self.ser.write(cmd.encode())

    # Equivalent to control_time_rotate()
    def control_time_rotate(self, servo_id, angle, t):
        """
        angle: degrees (0-180)
        t: seconds
        """
        time_ms = int(t * 1000)
        cmd = f"M,{servo_id},{angle},{time_ms}\n"
        self.ser.write(cmd.encode())

    def close(self):
        self.ser.close()

# test_servo = ArduinoServo()
# test_servo.trq_set(1,1)
# test_servo.control_time_rotate(1,90,0.01)

# test_servo.trq_set(2,1)
# test_servo.control_time_rotate(2,90,0.01)

# test_servo.trq_set(3,1)
# test_servo.control_time_rotate(3,90-15,0.01)

