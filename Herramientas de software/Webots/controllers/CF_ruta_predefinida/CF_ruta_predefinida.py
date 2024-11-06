# -*- coding: utf-8 -*-
# MIT License

from controller import Robot
from math import sqrt
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 0.4  # Altura de vuelo en metros
WAYPOINT_THRESHOLD = 0.05  # Distancia mínima para considerar que el dron ha alcanzado un punto

# Definir la trayectoria de puntos (x, y, z)
TRAJECTORY_POINTS = [
    [0.0, 0.0, FLYING_ATTITUDE],  # Punto inicial para despegue
    [0.0, 0.5, FLYING_ATTITUDE]   # Punto objetivo
]

def calculate_velocity_to_waypoint(waypoint, current_position):
    """Calcula la velocidad necesaria para moverse hacia el punto objetivo."""
    dx = waypoint[0] - current_position[0]
    dy = waypoint[1] - current_position[1]
    dz = waypoint[2] - current_position[2]
    
    return dx, dy, dz

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Inicializar motores
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(0)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(0)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(0)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(0)

    # Inicializar Sensores
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    # Inicializar variables
    past_time = 0
    first_time = True

    # Controlador PID para la velocidad del Crazyflie
    PID_crazyflie = pid_velocity_fixed_height_controller()
    current_waypoint_index = 0
    z_desired = FLYING_ATTITUDE

    print("\n====== Despegue y seguimiento de la trayectoria ======\n")

    # Bucle principal
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        dt = current_time - past_time

        # Obtener datos del sensor
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        y_global = gps.getValues()[1]
        z_global = gps.getValues()[2]  # Altura actual

        # Variables de control
        x_velocity_desired = 0
        y_velocity_desired = 0
        z_velocity_desired = 0
        yaw_desired = 0

        # Lógica para seguir la trayectoria de puntos
        if first_time:
            # Iniciar el despegue
            if z_global < FLYING_ATTITUDE - 0.05:
                z_velocity_desired = 0.1  # Subir lentamente hasta alcanzar la altura deseada
            else:
                first_time = False
                print("Altura alcanzada. Iniciando el seguimiento de la trayectoria...")
        else:
            # Verificar si se ha alcanzado el punto actual
            waypoint = TRAJECTORY_POINTS[current_waypoint_index]
            distance_to_waypoint = sqrt((waypoint[0] - x_global) ** 2 +
                                        (waypoint[1] - y_global) ** 2 +
                                        (waypoint[2] - z_global) ** 2)
            
            if distance_to_waypoint < WAYPOINT_THRESHOLD:
                # Avanzar al siguiente punto de la trayectoria
                current_waypoint_index += 1
                if current_waypoint_index >= len(TRAJECTORY_POINTS):
                    print("Trayectoria completada. Iniciando aterrizaje...")
                    break  # Salir del bucle principal
                else:
                    print(f"Alcanzado punto {current_waypoint_index}. Siguiendo al siguiente...")

            # Calcular las velocidades deseadas para alcanzar el siguiente punto
            waypoint = TRAJECTORY_POINTS[current_waypoint_index]
            x_velocity_desired, y_velocity_desired, z_velocity_desired = calculate_velocity_to_waypoint(
                waypoint, [x_global, y_global, z_global]
            )

        # Ajustar la altura deseada
        z_desired = z_global + z_velocity_desired * dt

        # Controlador PID para mantener la altura y velocidad deseada
        motor_power = PID_crazyflie.pid(dt, x_velocity_desired, y_velocity_desired,
                                        yaw_desired, z_desired,
                                        roll, pitch, yaw_rate,
                                        z_global, 0, 0)

        # Ajustar la velocidad de los motores
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        # Actualizar el tiempo pasado
        past_time = current_time
