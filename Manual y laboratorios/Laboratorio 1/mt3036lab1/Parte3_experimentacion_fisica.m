% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Laboratorio: Control de altura del dron Crazyflie 2.1
% =========================================================================
%% Tercera parte: Experimentación en el dron Crazyflie
% En este archvio se presenta el algoritmo para modificar el controlador
% PID de altura del dron Crazyflie junto a los comandos para captura de
% datos empleando el sistema de captura de movimiento del Robotat.

%% Familiarización con el MoCap del Robotat
% Función para establecer la conexión con el MoCap
robotat = robotat_connect();

% Función para obtener la pose de un marcador reflectivo
pose = robotat_get_pose(robotat, agent_id, "eulxyz");

% Función para cerrar la conexión con el MoCap
robotat_disconnect(robotat);

%% Familiarización con funciones de control del Crazyflie
% Función para establecer conexión con el Crazyflie
crazyflie_1 = crazyflie_connect(drone_number);

% Función para modificar el controlador PID de altura del Crazyflie
crazyflie_set_pid_z(Kp, Ki, Kd);

% Función para despegar al dron Crazylie
crazyflie_takeoff(crazyflie_1, height, duration);

% Función para aterrizar el dron
crazyflie_land(crazyflie_1);

% Función para cerrar la conexión con el Crazyflie
crazyflie_disconnect(crazyflie_1);

%% Algoritmo para experimentación con el dron crazyflie
% Establecer conexión con el sistema MoCap y el dron Crazyflie
drone_number = 8; % Corresponde al número del dron proporcionado
agent_id = 50; % Corresponde al marcador reflectivo del dron proporcionado
crazyflie_1 = crazyflie_connect(drone_number);
robotat = robotat_connect();

% Modificación del controlador
Kp = 4.0; 
Ki = 2.0;
Kd = 1.5;
crazyflie_set_pid_z(Kp, Ki, Kd);

% Pose inicial
pose = robotat_get_pose(robotat, agent_id, "eulxyz");

% Inicio de toma de datos con función Callback
% Colocar callback

% Secuencia de vuelo
% Despegue
crazyflie_takeoff(crazyflie_1, 1.0, 0.75);
% Tiempo de vuelo
pause(8);
% Aterrizaje
crazyflie_land(crazyflie_1);

% Desconexión
crazyflie_disconnect(crazyflie_1);
robotat_disconnect(robotat);

% Detener la captura de datos de la función callback


%% Graficar los datos obtenidos
% Graficar resultados
figure;
subplot(2, 1, 1);
plot(time, h_values, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Altura (m)');
title('Simulación de la altura del dron con PID');
grid on;

subplot(2, 1, 2);
plot(time, error_values, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Error (m)');
title('Error de Altura');
grid on;