% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Laboratorio: Control de altura del dron Crazyflie 2.1 
% =========================================================================
%% Segunda parte: Modelo simplificado en condiciones no ideales
m = 0.023;  % masa del dron en kg
g = 9.81;   % gravedad en m/s^2
c_drag = 0.15;  % Coeficiente de resistencia al aire
tau = 0.1;  % Constante de tiempo para la dinámica de los motores

% Parámetros del controlador PID
Kp = 2.5;  % Ganancia proporcional
Ki = 0.5;  % Ganancia integral
Kd = 0.1;  % Ganancia derivativa

% Altura objetivo
h_objetivo = 1; % Altura deseada en metros

% Tiempo de simulación
t_final = 5;  % Duración en segundos (más largo para observar mejor el comportamiento)
dt = 0.01;    % Intervalo de tiempo

% Variables iniciales
h = 0;  % Altura inicial
v = 0;  % Velocidad inicial
integral_error = 0;
prev_error = 0;
u = 0;  % Control inicial
u_max = 2.5 * m * g;  % Límite superior de la señal de control
u_min = 0;  % Límite inferior de la señal de control

% Vectores para guardar resultados
time = 0:dt:t_final;
h_values = zeros(size(time));
v_values = zeros(size(time));
u_values = zeros(size(time));  % Para graficar la señal de control
error_values = zeros(size(time));  % Para graficar el error

% Bucle de simulación
for i = 1:length(time)
    % Cálculo del error
    error = h_objetivo - h;
    
    % Controlador PID
    integral_error = integral_error + error * dt;
    derivative_error = (error - prev_error) / dt;
    u_controlador = Kp * error + Ki * integral_error + Kd * derivative_error;
    prev_error = error;
    
    % Dinámica del motor con retardo
    u = u + (u_controlador - u) * (dt / tau);
    
    % Limitar la señal de control (saturación de los motores)
    u = max(min(u, u_max), u_min);
    
    % Aceleración con resistencia al aire
    a = (u - m * g - c_drag * v) / m;  % Aceleración incluyendo resistencia del aire
    
    % Actualizar velocidad y altura
    v = v + a * dt;
    h = h + v * dt;
    
    % Guardar valores para graficar
    h_values(i) = h;
    v_values(i) = v;
    u_values(i) = u;
    error_values(i) = error;
end

% Graficar resultados
figure;
subplot(3, 1, 1);
plot(time, h_values, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Altura (m)');
title('Simulación de la altura del dron (modelo realista)');
grid on;

subplot(3, 1, 2);
plot(time, error_values, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Error (m)');
title('Error de Altura');
grid on;

subplot(3, 1, 3);
plot(time, u_values, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Señal de Control');
title('Señal de Control con Saturación');
grid on;
