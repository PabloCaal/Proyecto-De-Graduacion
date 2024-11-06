% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Laboratorio: Seguimiento de trayectoria a través de una ruta con 
% obstáculos con el dron Crazyflie 2.1
% =========================================================================
%% ========================================================================
% Primera parte: Sistema de captura de movimiento Robotat
% =========================================================================
%% Añadir al path las carpetas de comandos usando una ruta relativa
addpath('../../Crazyflie-Matlab');

%% Uso del sistema de captura de movimiento
% Función para crear la conexión con el sistema Robotat 
robotat = robotat_connect();

% Función para obtener la pose de un marcador reflectivo
agent_id = 50; % Número del marcador reflectivo seleccionado
pose = robotat_get_pose(robotat, agent_id, "eulxyz");

% Función para cerrar la conexión con el sistema Robotat
robotat_disconnect(robotat);

%% Captura de datos durante tiempo indefinido
% En esta sección deberá desarrollar un algoritmo que se conecte al sistema
% Robotat y capture datos cíclicamente durante un tiempo indefinido y los 
% almacene en un arreglo llamado pose.

robotat = robotat_connect(); % Abrir conexión

% Desarrolle un ciclo de captura de datos con tiempo indefinido
pose = [];

% . . . 

robotat_disconnect(robotat); % Cerrar conexión

%% ========================================================================
% Segunda parte: Generación de trayectoria a través de obstáculos
% =========================================================================
%% Lectura de marcadores para generar la trayectoria
% En esta sección deberá desarrollar un algoritmo para capturar los datos
% de pose de los marcadores reflectivos de los puntos de despegue y
% aterrizaje, junto con los obstáculos para generar una trayectoria.

robotat = robotat_connect(); % Abrir conexión

% Define el número de obstáculos
N = 3; % Cambia este valor según el número de obstáculos
obstacle_id = [11, 12, 13]; % Arreglo con id de los markers de obstáculos

% Inicialización de las matrices de puntos
initial_point = robotat_get_pose(robotat, 10, "eulxyz");
final_point = robotat_get_pose(robotat, 14, "eulxyz");
obstacle_point = zeros(N, 6); % Crear una matriz para almacenar la pose de N obstáculos

% Captura la pose de los obstáculos de forma dinámica
for i = 1:N
    obstacle_point(i,:) = robotat_get_pose(robotat, obstacle_id(i), "eulxyz"); % IDs secuenciales
end

robotat_disconnect(robotat); % Cerrar conexión

%% Prueba temporal con valores genéricos
% Número de obstáculos en la trayectoria
N = 3;
obstacle_id = [11, 12, 13]; % Id de marcadores de los obstáculos 

% Lectura de poses de los puntos clave para la trayectoria
initial_point = [0, 0.3, 0.0, 0, 0, 0]; 
obstacle_point = zeros(N, 6); % Crear una matriz para almacenar la pose de N obstáculos
final_point = [0, 2.1, 0.0, 0, 0, 0];
obstacle_point(1,:) = [-0.2, 0.6, 0.5, 0, 0, 90];
obstacle_point(2,:) = [0.2, 1.2, 0.5, 0, 0, 90];
obstacle_point(3,:) = [-0.2, 1.8, 0.5, 0, 0, 90];

%% Visualización de la trayectoria
takeoff_point = initial_point(1:3) + [0, 0, 0.3];
land_point = final_point(1:3) + [0, 0, 0.3];

% Graficar los puntos relevantes de la trayectoria en un espacio 3D
figure;
hold on;
[x_floor, y_floor] = meshgrid(-2:0.1:2, -2.5:0.1:2.5);
z_floor = zeros(size(x_floor));
surf(x_floor, y_floor, z_floor, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5); % Dibujar el piso
% Graficar los puntos: inicio, despegue, aterrizaje y final
plot3(initial_point(1), initial_point(2), initial_point(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
% text(initial_point(1), initial_point(2), initial_point(3), 'Origin', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot3(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
% text(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'Takeoff', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot3(land_point(1), land_point(2), land_point(3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
% text(land_point(1), land_point(2), land_point(3), 'Land Point', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot3(final_point(1), final_point(2), final_point(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% text(final_point(1), final_point(2), final_point(3), 'Final Point', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Generar los obstáculos y graficarlos en el espacio 3D
% Geometría del obstáculo
diameter = 0.34;
radius = diameter / 2;
obstacle_distance = 0.15; % Distancia pre y post obstáculos
theta = linspace(0, 2*pi, 7); % Generar el hexágono

% Función para aplicar la orientación adecuada
apply_yaw = @(x, y, yaw) [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [x; y];

% Arreglos para almacenar puntos importantes
obstacle_center = zeros(N, 6); % Puntos de centro de obstáculos
pre_obstacle = zeros(N, 3); % Puntos pre-obstáculos
post_obstacle = zeros(N, 3); % Puntos post-obstáculos

% Ciclo para determinar puntos pre, central y post obstáculos
for i = 1:size(obstacle_point,1)
    obstacle_center(i,1:3) = obstacle_point(i,1:3) - [0, 0, radius/2];

    yaw_obstacle = deg2rad(obstacle_point(i,6));  
    hexagon_y = radius * cos(theta); 
    hexagon_z = radius * sin(theta) + obstacle_center(i,3);
    rotated_hexagon = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle);

    pre_obstacle(i,:) = obstacle_center(i,1:3) - obstacle_distance*[cos(yaw_obstacle), sin(yaw_obstacle), 0];
    post_obstacle(i,:) = obstacle_center(i,1:3) + obstacle_distance*[cos(yaw_obstacle), sin(yaw_obstacle), 0];

    % Graficar los obstáculos en el espacio 3D
    plot3(obstacle_point(i,1)+rotated_hexagon(1,:), obstacle_point(i,2)+rotated_hexagon(2,:), hexagon_z, 'r', 'LineWidth', 2);    
end

% Generación de los puntos de la trayectoria a través de los obstáculos
trajectory_points = takeoff_point;

% Añadir puntos pre, centro y post de cada obstáculo
for i = 1:N
    trajectory_points = [trajectory_points; pre_obstacle(i,:); post_obstacle(i,:)];
end

% Finalmente añadimos el punto de aterrizaje
trajectory_points = [trajectory_points; land_point];
% Interpolación de la trayectoria
n_interp = 20; % Número de puntos interpolados
t = 1:size(trajectory_points, 1);
t_interp = linspace(1, t(end), n_interp); 
x_interp = interp1(t, trajectory_points(:, 1), t_interp, 'pchip');
y_interp = interp1(t, trajectory_points(:, 2), t_interp, 'pchip');
z_interp = interp1(t, trajectory_points(:, 3), t_interp, 'pchip');

% Figura 3D para la previsualización

% Graficar la trayectoria interpolada
plot3(x_interp, y_interp, z_interp, 'm-', 'LineWidth', 2);
plot3(x_interp, y_interp, z_interp, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Puntos interpolados

% Configurar los ejes
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Visualización de la trayectoria generada');
grid on;
axis equal;
xlim([-1 1]); % Ajustar límites en X
ylim([-0.3 2.4]); % Ajustar límites en Y
zlim([0 1]); % Ajustar límites en Z para ver desde el suelo hacia arriba
view(3);
hold off;


%%
% Utilizando los datos anteriores, deberá desarrollar una trayectoria de
% puntos que inicie 0.5 metros sobre el initial_point y termine 0.5 metros
% sobre el final_point. 



%% Previsualización de la trayectoria
% Esta subsección la utilizará para generar una visualización 3D de la
% trayectoria generada en una representación simple del entorno de
% experimentación



%% ========================================================================
% Tercera parte: Seguimiento de trayectoria en Robotat con Crazyflie
% =========================================================================
%% Añadir al path las carpetas de comandos usando una ruta relativa
addpath('../../Crazyflie-Matlab');

%% Ejecución de seguimiento de trayectoria en Robotat
velocity = 1;
% Despegue
crazyflie_takeoff(crazyflie_1);
pause(1);
% Seguimiento de la trayectoria
crazyflie_trayectory_robotat(crazyflie, x, y, z, velocity, robotat, agent_id)
pause(1);
% Aterrizaje
crazyflie_land(crazyflie_1);
% Desconexión de Robotat y Crazyflie
crazyflie_disconnect(crazyflie_1);
robotat_disconnect(robotat);