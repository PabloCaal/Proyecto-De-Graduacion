% =========================================================================
%                     MT3006 - LABORATORIO 2: 
% Generación y seguimiento de trayectorias con drones Crazyflie 2.1
% -------------------------------------------------------------------------
% Puede ver el detalle de los pasos en la guía adjunta, pero se le deja un
% resumen:
% 0. Reciba las instrucciones generales al inicio del laboratorio.
% 1. Seleccione un dron Crazyflie a utilizar. Este y el siguiente 
% laboratorio se estarán trabajando en PAREJAS.
% 2. Verifique que puede conectarse al dron de forma inalámbrica y que 
% puede obtener correctamente su pose dentro del Robotat.
% 3. Utilice la información del sistema de captura para definir, en donde
% se indica, los obstáculos y puntos relevantes (despegue y aterrizaje).
% Con ello, genere trayectorias evaluando los distintos métodos de
% interpolación dispoibles y verifique que la visualización de la 
% trayectoria coincide con lo que observa sobre la plataforma del Robotat.
% 4. Experimente y familiaricese con el dron Crazyflie empleando las 
% funciones de control de alto nivel.
% 5. Combine el resultado de la generación de trayectorias con las 
% funciones de control del dron Crazyflie para ejecutar físicamente el 
% seguimiento de la trayectoria.
% 6. Verifique que la generación y ejecución continúa funcionando
% adecuadamente variando la posición del punto de aterrizaje y la 
% distribución física de los obstáculos.
% =========================================================================
%% Añadir funciones Robotat y Crazyflie al path
addpath('../Robotat');
addpath('../Crazyflie');

%% Conexión al Robotat 
robotat = robotat_connect();

%% Conexión al agente Crazyflie 2.1
robot_no = 50; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot
robot = crazyflie_connect(robot_no);

%% Definición de obstáculos y puntos relevantes (despegue y aterrizaje)
% Se definen uno por uno los obstáculos, en caso de querer deshabilitar
% alguno igualar a un vector vacío
obs1 = [3.5*(rand-0.5), 4.5*(rand-0.5)];
obs2 = [3.5*(rand-0.5), 4.5*(rand-0.5)];
obs3 = [3.5*(rand-0.5), 4.5*(rand-0.5)];
obs4 = [3.5*(rand-0.5), 4.5*(rand-0.5)];
obs5 = [3.5*(rand-0.5), 4.5*(rand-0.5)];
obs6 = [3.5*(rand-0.5), 4.5*(rand-0.5)];
obs7 = obs6 + 0.05;
obs8 = [];
obs9 = [];
obs10 = [];

% Array de obstáculos
obs = [obs1; obs2; obs3; obs4; obs5; obs6; obs7; obs8; obs9; obs10];

%% GENERACIÓN DE TRAYECTORIA
% Emplee las funciones de la Robotics Toolbox de Peter Corke junto con el
% documento Navigation-PC para encontrar una ruta óptima entre la posición
% actual del robot y la meta establecida mediante el algoritmo D*. Tome en 
% consideración lo siguiente:


%% VISUALIZACIÓN DE TRAYECTORIA EN RUTA CON OBSTÁCULOS


%% Paro de emergencia para el robot (importante tener a la mano)
crazyflie_stop(robot);

%% SEGUIMIENTO DE TRAYECTORIA CON FUNCIONES CRAZYFLIE DE ALTO NIVEL 
% Para la ejecución de la planificación, se le recomienda implementar una
% estrategia de pursuit para el controlador, en donde (xg, yg) sea un punto
% dentro de la ruta óptima y este valor se vaya actualizando conforme el
% robot se acerque al punto. Es decir, puede emplearse una condición sobre
% el error de posición para actualizar el valor de (xg, yg) hasta acabarse
% los puntos dentro de la ruta.


%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Crazyflie 2.1
crazyflie_disconnect(robot);