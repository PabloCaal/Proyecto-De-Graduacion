from controller import Robot
from utilities import * 
import numpy as np
import cv2
robot = Robot() 

sensors = ["gps", "imu", "gyro"]
sensors = Sensors(robot, sensors)
motors = get_crazyflie_motors(robot) 
timeStep = int(robot.getBasicTimeStep()) 
sensors.enable_sensors(timeStep)

camera = robot.getDevice("cam")
camera.enable(timeStep)
camera.recognitionEnable(timeStep)

roll_pid = PID(robot, 0.5, 0.0001, 0.5) 
pitch_pid = PID(robot, 0.5, 0.0001, 0.5)
x_pid = PID(robot, 0.25, 0.0001, 0.75)
y_pid = PID(robot, 0.25, 0.0001, 0.75)
yaw_pid = PID(robot, 1.5, 0.0001, 2)
z_pid = PID(robot, 12, 0.001, 10)

pids = {
"roll": roll_pid,
"pitch": pitch_pid,
"x": x_pid,
"y": y_pid,
"yaw": yaw_pid,
"z": z_pid
}


is_done = False
distance_offset = 0.03
yaw_offset = np.deg2rad(0.2)


time_start_stab = 0
stab_dur = 3

states = [State(0,0,0.75,0,0,0,0,0,0),State(2,10,10,0,0,0,0,0,0)]
while robot.step(timeStep) != -1 and is_done == False:
    curr_state = getState(sensors) 
    write_state(curr_state)  
    is_done, states, time_start_stab = go_states(robot, motors, sensors, states, distance_offset, yaw_offset, pids, time_start_stab, stab_dur)
    print("States remaining: " + str(len(states)))
    if(len(states) == 0):
        set_motor_velocities(motors, 0, 0, 0, 0)
    pass
    

