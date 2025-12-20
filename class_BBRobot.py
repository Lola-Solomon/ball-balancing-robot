import class_servo as cs
import math
import time

class BBrobot:
    # Create the structure of the robot
    def __init__(self, ids):  # Receives a list of servo IDs
        self.ids = ids 
        # Prepare the servos
        self.servos = cs.ArduinoServo()
        # Link lengths L = [base, lower link, upper link, top]
        self.L = [0.05, 0.045, 0.095, 0.08]
        # Initial posture (theta, phi, pz)
        self.ini_pos = [0, 0, 0.11]
        self.pz_max = 0.11
        self.pz_min = 0.1
        self.phi_max = 15

    # Method to prepare the robot
    def set_up(self):
        # Turn servo torque ON
        self.servos.trq_set(self.ids[0],1)
        self.servos.trq_set(self.ids[1],1)
        self.servos.trq_set(self.ids[2],1)
        
    # Method to clean up the robot
    def clean_up(self):
        # Turn servo torque OFF
        self.servos.trq_set(self.ids[0],0)
        self.servos.trq_set(self.ids[1],0)
        self.servos.trq_set(self.ids[2],0)
        # Close the serial port
        self.servos.close()

    # Method to calculate inverse kinematics
    def kinema_inv(self, n, Pz):
        L = self.L
        # Calculate the height relative to the servo reference (Pmz ± inversion)
        A = (L[0] + L[1]) / Pz
        B = (Pz**2 + L[2]**2 - (L[0] + L[1])**2 - L[3]**2) / (2 * Pz)
        C = A**2 + 1
        D = 2 * (A * B - (L[0] + L[1]))
        E = B**2 + (L[0] + L[1])**2 - L[2]**2
        Pmx = (-D + math.sqrt(D**2 - 4*C*E)) / (2 * C)
        Pmz = math.sqrt(L[2]**2 - Pmx**2 + 2*(L[0] + L[1])*Pmx - (L[0] + L[1])**2)

        # Derive angle of servo A
        a_m_x = (L[3] / (math.sqrt(n[0]**2 + n[2]**2))) * (n[2])
        a_m_y = 0
        a_m_z = Pz + (L[3] / (math.sqrt(n[0]**2 + n[2]**2))) * (-n[0])
        A_m = [a_m_x, a_m_y, a_m_z]
        A = (L[0] - A_m[0]) / A_m[2]
        B = (A_m[0]**2 + A_m[1]**2 + A_m[2]**2 - L[2]**2 - L[0]**2 + L[1]**2) / (2 * A_m[2])
        C = A**2 + 1
        D = 2 * (A * B - L[0])
        E = B**2 + L[0]**2 - L[1]**2
        ax = (-D + math.sqrt(D**2 - 4*C*E)) / (2 * C)
        ay = 0
        az = math.sqrt(L[1]**2 - ax**2 + 2*L[0]*ax - L[0]**2)
        if (a_m_z < Pmz):
            az = -az
        A_2 = [ax, ay, az]
        theta_a = 90 - math.degrees(math.atan2(A_2[0] - L[0], A_2[2]))

        # Derive angle of servo B
        b_m_x = (L[3] / (math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1]))) * (-n[2])
        b_m_y = (L[3] / (math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1]))) * (-math.sqrt(3)*n[2])
        b_m_z = Pz + (L[3] / (math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1]))) * (math.sqrt(3)*n[1] + n[0])
        B_m = [b_m_x, b_m_y, b_m_z]

        A = -(B_m[0] + math.sqrt(3)*B_m[1] + 2*L[0]) / B_m[2]
        B = (B_m[0]**2 + B_m[1]**2 + B_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2 * B_m[2])
        C = A**2 + 4
        D = 2*A*B + 4*L[0]
        E = B**2 + L[0]**2 - L[1]**2
        x = (-D - math.sqrt(D**2 - 4*C*E)) / (2 * C)
        y = math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)
        if (b_m_z < Pmz):
            z = -z
        B_2 = [x, y, z]
        theta_b = 90 - math.degrees(math.atan2(math.sqrt(B_2[0]**2 + B_2[1]**2) - L[0], B_2[2]))

        # Derive angle of servo C
        c_m_x = (L[3] / (math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1]))) * (-n[2])
        c_m_y = (L[3] / (math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1]))) * (math.sqrt(3)*n[2])
        c_m_z = Pz + (L[3] / (math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1]))) * (-math.sqrt(3)*n[1] + n[0])
        C_m = [c_m_x, c_m_y, c_m_z]

        A = -(C_m[0] - math.sqrt(3)*C_m[1] + 2*L[0]) / C_m[2]
        B = (C_m[0]**2 + C_m[1]**2 + C_m[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2 * C_m[2])
        C = A**2 + 4
        D = 2*A*B + 4*L[0]
        E = B**2 + L[0]**2 - L[1]**2
        x = (-D - math.sqrt(D**2 - 4*C*E)) / (2 * C)
        y = -math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2)
        if (c_m_z < Pmz):
            z = -z
        C_2 = [x, y, z]
        theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0]**2 + C_2[1]**2) - L[0], C_2[2]))

        thetas = [theta_a, theta_b, theta_c]
        return thetas

    # Method to achieve posture (theta, phi, Pz) after t seconds
    def control_t_posture(self, pos, t):
        theta = pos[0]
        phi = pos[1]
        # Motion constraints
        if phi > self.phi_max:
            phi = self.phi_max
        Pz = pos[2]
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min

        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        n = [x, y, z]
        angles = self.kinema_inv(n, Pz)

        self.servos.control_time_rotate(self.ids[0],angles[0], t)
        self.servos.control_time_rotate(self.ids[1],angles[1], t)
        self.servos.control_time_rotate(self.ids[2],angles[2]-3.9, t)

        time.sleep(t)
    
    def Initialize_posture(self):
        pos = self.ini_pos
        t = 1
        self.control_t_posture(pos, t)

test_pos = BBrobot([1,2,3])
test_pos.set_up()
test_pos.Initialize_posture()

# test_pos.control_t_posture([0,-15,0.11],0.05)

# for i in range(-15,15):
#     test_pos.control_t_posture([0,i,0.15],0.05)
#     # time.sleep(2)

# for i in range(0,360):
#     test_pos.control_t_posture([i,15,0.12],0.05)
#     # time.sleep(2)