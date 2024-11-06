from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard 
import numpy as np
import math

motor_names = ["m1_motor","m2_motor","m3_motor","m4_motor"]

class Sensors: 
    def __init__(self,robot, input_sensors): 
        self.sensors = {}
        print(self.sensors)
        for sensor in input_sensors:
            self.sensors[sensor] = robot.getDevice(sensor)
    
    def enable_sensors(self, timeStep): 
        sensors_used = self.sensors.keys()
        for sensor in sensors_used:
            self.sensors[sensor].enable(timeStep)
        
    def get_sensor_values(self, sensor):
        if sensor == "imu":
            roll, pitch, yaw = self.sensors["imu"].getRollPitchYaw()[0], self.sensors["imu"].getRollPitchYaw()[1], self.sensors["imu"].getRollPitchYaw()[2] 
            return roll, pitch, yaw
        else:
            return self.sensors[sensor].getValues()[0], self.sensors[sensor].getValues()[1], self.sensors[sensor].getValues()[2]
class State: 
    def __init__(self, x, y, z, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_rate = roll_rate 
        self.pitch_rate = pitch_rate
        self.yaw_rate = yaw_rate
    
    def print_state(self):
        print("X: " + str(self.x) + " " + "Y: " + str(self.y) + " " + "Z: " + str(self.z))
        print("Roll: " + str(self.roll)  + " " + "Pitch: " + str(self.pitch)  + " " + "Yaw: " + str(self.yaw))
    
    def __sub__(self, other):
        return State(self.x-other.x, self.y-other.y, self.z-other.z, self.roll-other.roll, self.pitch-other.pitch, self.yaw-other.yaw, self.roll_rate - other.roll_rate, self.pitch_rate - other.pitch_rate, self.yaw_rate-other.yaw_rate)
    
    def __eq__(self, other):
        if (self.x == other.x and self.y == other.y and self.z == other.z and self.yaw == other.yaw and self.pitch == other.pitch and self.roll == other.roll):
            return True
        else:
            return False
class PID: 
    def __init__(self, robot, kp, ki, kd):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.I = 0 
        self.prev_t = 0 
        self.t = 0
        self.prev_e = 0 
        self.robot = robot 
        
    def give_output(self, error): # Bash tou error 
        self.t = self.prev_t + 0.032
        output_kp = error*self.kp 
        output_kd = self.kd*(error-self.prev_e)/(self.t-self.prev_t)
        output_ki = self.I + self.ki*error*(self.t-self.prev_t)
        self.I = output_ki
        self.prev_t = self.t
        self.prev_e = error
        return (output_kp+output_kd+output_ki)
     
    def reset_pid(self):
        self.prev_t = 0
        self.t = 0
        self.I = 0
        self.prev_e = 0

def resetPIDs(pid_array):
    keys = pid_array.keys()
    for pid_key in keys:
        pid_array[pid_key].reset_pid()
      
def get_crazyflie_motors(robot): 
    global motor_names 
    motor_arr = [] 
    for i in range(0,len(motor_names)): 
        motor_arr.append(robot.getDevice(motor_names[i])) 
        motor_arr[i].setPosition(float('inf'))
        motor_arr[i].setVelocity(0)  
    return motor_arr

def clamp(var, lower, upper):
    if (var > upper):
        var = upper
        return var
    if (var < lower):
        var = lower
        return var
    return var

def getState(sensors):
    return State(*sensors.get_sensor_values("customGPS"), *sensors.get_sensor_values("imu"), *sensors.get_sensor_values("gyro"))

"""
def transform_state(des_state, curr_state):
    c_phi = math.cos(curr_state.pitch)
    s_phi = math.sin(curr_state.pitch)
    c_theta = math.cos(curr_state.roll)
    s_theta = math.sin(curr_state.roll)
    c_psi = math.cos(curr_state.yaw)
    s_psi = math.sin(curr_state.yaw)
    first_row [c_theta*c_psi,-c_phi*s_psi+s_phi*s_theta*c_psi,s_phi*s_psi+c_phi*s*theta*c_psi,curr_state.x]
    second_row [c_theta*s_psi,c_phi*c_psi+s_phi*s_theta*s_psi,]
    third_row = [-s_theta, s_phi*c_theta, c_phi*c_thetta]
    fourth_row = [0,0,0,1]
    trans_matrix = np.ndarray(first_row,second_row,third_row,fourth_row)
"""

def motor_mixing(motor_arr, curr_state, desired_state, pids): 
    hover_vel = 55.3 
    error_state =  desired_state-curr_state 
    error_state.x = (desired_state.x - curr_state.x)*math.cos(curr_state.yaw) - (desired_state.y - curr_state.y)*math.sin(curr_state.yaw)
    error_state.y = (desired_state.y - curr_state.y)*math.cos(curr_state.yaw) + (desired_state.x - curr_state.x)*math.sin(curr_state.yaw)
    error_state.print_state()
    pitch_des = clamp(pids["x"].give_output(error_state.x), -np.deg2rad(10), np.deg2rad(10))
    roll_des = -clamp(pids["y"].give_output(error_state.y), -np.deg2rad(10), np.deg2rad(10))
    error_state.pitch = pitch_des - curr_state.pitch
    error_state.roll = roll_des - curr_state.roll
    z_pid_output = pids["z"].give_output(error_state.z)
    roll_pid_output = clamp(pids["roll"].give_output(error_state.roll), -np.deg2rad(10), np.deg2rad(10))
    pitch_pid_output = clamp(pids["pitch"].give_output(error_state.pitch), -np.deg2rad(10), np.deg2rad(10))
    yaw_pid_output = pids["yaw"].give_output(error_state.yaw)
    v1 = hover_vel + z_pid_output - roll_pid_output - pitch_pid_output + yaw_pid_output
    v2 = hover_vel + z_pid_output - roll_pid_output + pitch_pid_output - yaw_pid_output
    v3 = hover_vel + z_pid_output + roll_pid_output + pitch_pid_output + yaw_pid_output
    v4 = hover_vel + z_pid_output + roll_pid_output - pitch_pid_output - yaw_pid_output
    v1 = clamp(-v1,-600,600)
    v2 = clamp(v2,-600,600)
    v3 = clamp(-v3,-600,600)
    v4 = clamp(v4,-600,600)
    print("V1: " + str(v1) + " " + "V2: " + str(v2) + " " +"V3: " + str(v3) + " " + "V4: " + str(v4))
    motor_velocities = (v1, v2, v3, v4)
    set_motor_velocities(motor_arr, *motor_velocities)
    
def set_motor_velocities(motor_arr, v1, v2, v3, v4): 
    motor_arr[0].setVelocity(v1)
    motor_arr[1].setVelocity(v2)
    motor_arr[2].setVelocity(v3)
    motor_arr[3].setVelocity(v4)

def euclideanDist(state1,state2):
    return np.sqrt((state1.x-state2.x)**2 + (state1.y-state2.y)**2 + (state1.z-state2.z)**2)

def go_states(robot, motor_arr, sensors, states_arr, dist_offset, yaw_offset, pids, stab_time, stab_dur):
    curr_state = getState(sensors) 
    if len(states_arr) == 0: 
        return True, [], 0 
    if(euclideanDist(curr_state,states_arr[0]) > dist_offset or abs(curr_state.yaw-states_arr[0].yaw) > yaw_offset): 
        motor_mixing(motor_arr, curr_state, states_arr[0], pids)
        return False, states_arr, float(robot.getTime()) 
    else:
        if(float(robot.getTime()) - stab_time >= stab_dur): 
            states_arr = states_arr[1:]
            print("Attained state") 
            resetPIDs(pids)
            return False, states_arr, float(robot.getTime()) 
        else:
            motor_mixing(motor_arr, curr_state, states_arr[0], pids) 
            return False, states_arr, stab_time
"""
def land(motors, sensors, pid, offset):
    
    hover_vel = 55
    curr_state = getState(sensors)
    error_state = State(0,0,0-curr_state.z,0,0,0,0,0,0)
    output = pid.give_output(error_state.z)
    if(error_state.z > offset):
        v1 = -(hover_vel + output)
        v2 = hover_vel + output
        v3 = -(hover_vel + output)
        v4 = hover_vel + output
    else:
        print("landed")
        v1 = 0
        v2 = 0
        v3 = 0
        v4 = 0
        
    set_motor_velocities(motors, v1, v2, v3, v4)
"""
def write_state(state):
    filename = "flight_experiment.txt"
    with open(filename, "a") as results_file:
        results_file.write(str(state.x) + "," + str(state.y) + "," + str(state.z) + "\n")

def line_n_back(start_alt,final_pos, angles):
    State1 = State(0, 0, start_alt, angles[0], angles[1], angles[2], 0, 0, 0)
    State2 = State(final_pos[0], final_pos[1], final_pos[2], angles[0], angles[1], angles[2], 0, 0, 0)
    State3 = State1
    return [State1,State2,State3]

def line(start_alt, final_pos, angles):
    State1 = State(0, 0, start_alt, angles[0], angles[1], angles[2], 0, 0, 0)
    State2 = State(final_pos[0], final_pos[1], final_pos[2], angles[0], angles[1], angles[2], 0, 0, 0)
    return [State1, State2]

def circle(center, curr_state, angle, n = 30):
    alt = curr_state.z
    center = State(center[0], center[1], alt, 0, 0, 0, 0, 0, 0)
    radius = euclideanDist(curr_state, center)
    starting_angle = np.arctan2(curr_state.y-center.y, curr_state.x-center.x)
    thetas = np.linspace(starting_angle, starting_angle+angle, n)
    X = radius*np.cos(thetas) + center.x
    Y = radius*np.sin(thetas) + center.y
    states = []
    states.append(State(curr_state.x,curr_state.y,alt,0,0,0,0,0,0))
    for i in range(0,n):
        perimeter_point = State(X[i],Y[i],alt,0,0,0,0,0,0)
        states.append(perimeter_point)
    return states
 
def square(altitude, side1, side2, alt_change = 0, lock_yaw = True):
    state_1 = State(0,0,altitude,0,0,0,0,0,0)
    if lock_yaw:
        state_2 = State(side1,0,altitude+alt_change,0,0,0,0,0,0)
        state_3 = State(side1,side2,altitude-alt_change,0,0,0,0,0,0)
        state_4 = State(0,side2,altitude+alt_change,0,0,0,0,0,0)
        state_5 = State(0,0,altitude-alt_change,0,0,0,0,0,0)
        return [state_1,state_2,state_3,state_4,state_5]
    else:
        state_2 = State(side1,0,altitude+alt_change,0,0,0,0,0,0)
        state_3 = State(side1,0,altitude+alt_change,0,0,np.deg2rad(90),0,0,0)
        state_4 = State(side1,side2,altitude-alt_change,0,0,np.deg2rad(90),0,0,0)
        state_5 = State(0,side2,altitude+alt_change,0,0,np.deg2rad(180),0,0,0)
        state_6 = State(0,0,altitude-alt_change,0,0,np.deg2rad(-90),0,0,0)
        return [state_1,state_2,state_3,state_4,state_5,state_6]
    

def eight(alt, curr_state, size = 2, n = 20):
    t = np.linspace(0,2*np.pi, n)
    x = curr_state.x + size*np.sin(t)
    y = curr_state.y + size*np.sin(t)*np.cos(t)
    perimeter_points = []
    for i in range(0, n):
        point = State(x[i], y[i], alt, 0, 0, 0, 0, 0, 0)
        perimeter_points.append(point)
    return perimeter_points

def spline(des_state1,des_state2,des_state3, des_state4, n = 100):
    X = np.array([des_state1.x,des_state2.x,des_state3.x, des_state4.x])
    Y = np.array([des_state1.y,des_state2.y,des_state3.y, des_state4.y])
    Z = np.array([des_state1.z,des_state2.z,des_state3.z, des_state4.z])
    t = np.linspace(0,1,4)
    A = np.column_stack([t**3,t**2,t,np.ones_like(t)])
    coeffs_x = np.linalg.solve(A,X)
    coeffs_y = np.linalg.solve(A,Y)
    coeffs_z = np.linalg.solve(A,Z)
    t = np.linspace(0,1,n)
    X_poly = np.poly1d(coeffs_x)
    Y_poly = np.poly1d(coeffs_y)
    Z_poly = np.poly1d(coeffs_z)
    x = X_poly(t)
    y = Y_poly(t)
    z = Z_poly(t)
    print(len(X))
    traj_ret = []
    for i in range(0,len(x)):
        point = State(x[i],y[i],z[i],0,0,0,0,0,0)
        traj_ret.append(point)
    return traj_ret
    
