import serial
import time

class ArduinoServo:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
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
