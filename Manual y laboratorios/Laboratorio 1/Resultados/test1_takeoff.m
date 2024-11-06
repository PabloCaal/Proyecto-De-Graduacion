% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Prueba de Take Off y Land en Crazyflie con Flow Deck
% =========================================================================

%% Añadir al path las carpetas de comandos usando una ruta relativa
addpath('../Crazyflie-Matlab-Commands');
addpath('../Robotat-Matlab-Commands');

%% Ejecución de prueba de 9despegue y aterrizaje

dron_id = 8; 
crazyflie_1 = crazyflie_connect(dron_id);
pause(1);

crazyflie_takeoff(crazyflie_1);
pause(8);

crazyflie_land(crazyflie_1);
pause(1);

crazyflie_disconnect(crazyflie_1);