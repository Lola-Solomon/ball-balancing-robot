import math
import time

class pid:
    def __init__(self,v_pid,v,alpha):
        self.kp=v_pid[0] #proportional controlelr
        self.ki=v_pid[1] #integral controlelr
        self.kd=v_pid[2] #derivative controlelr
        self.v=v         #factor to convert the position to tilt angle

        self.alpha=alpha
        self.prev_output_x=0
        self.prev_output_y=0
        self.prev_error_x=0
        self.prev_error_y=0
        self.integral_x=0
        self.integral_y=0
        self.last_time=None
        self.count=0
        self.F=0


    def compute(self,goal,current_value):

        #calculating time to obtain dt
        current_time=time.pref_counter()
        if self.last_time is None:
            self.last_time is None=current_time
            return 0,0
        
        dt=(current_time - self.last_time)
        #proportional error
        error_x=goal[0]-current_value[0]
        error_y=goal[1]-current_value[1]
        #integral error
        self.integral_x+=error_x*dt
        self.integral_x+=error_x*dt
        #derivative error
        der_x=(error_x - self.prev_error_x)/dt
        der_y=(error_y - self.prev_error_y)/dt
        #implementing pid
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * der_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * der_y
        #low pass filter
        output_x = self.alpha * output_x + (1 - self.alpha) * self.prev_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.prev_output_y
        #the input to the IK solution
        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360
        phi = self.v* math.sqrt(output_x**2 + output_y**2)

        self.prev_error_x=error_x
        self.prev_error_y=error_y
        self.prev_output_x=output_x
        self.prev_output_y=output_y
        self.last_time=current_time

        return theta,phi

    
        
