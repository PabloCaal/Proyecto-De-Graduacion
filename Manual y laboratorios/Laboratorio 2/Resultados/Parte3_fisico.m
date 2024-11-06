% Extraer la columna Z
z_Robotat = pose_crazyflie(:, 3)-0.04;
N = length(z_Robotat);
t = (0:N-1)*0.1;

% Graficar la componente Z en función del tiempo
figure;
plot(t, z_Robotat, '-*');
grid on;
xlabel('Tiempo [s]');
ylabel('Z [m]');
title('Componente Z del marker en función del tiempo');

%% Filtrar datos entre 3 segundos y 6 segundos
indices = t >= 2.2 & t <= 5.1;  % Índices donde el tiempo está entre 3 y 6 segundos
tiempo_filtrado = t(indices) - 2.2;    % Vector de tiempo filtrado
lecturas_filtradas = z_Robotat(indices)*1.9;  % Datos filtrados
error = 1 - lecturas_filtradas;
% Graficar la componente Z en función del tiempo
figure;
subplot(2, 1, 1);
plot(tiempo_filtrado, lecturas_filtradas, 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('Z [m]');
title('Componente Z del marker en función del tiempo');

subplot(2, 1, 2);
plot(tiempo_filtrado, error, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Error (m)');
title('Error de Altura');
grid on;


h = lecturas_filtradas;
u = ones(size(h));

%% Identificación de sistemas
%stepinfo(h, t)
Ts = 0.1;  % Tiempo de muestreo (en segundos, ajusta según tus datos)
data = iddata(h, u, Ts);  % Crear el objeto de datos para identificación

np = 8; 
nz = 3; 

% Ajustar la función de transferencia
sys_closed = tfest(data, np, nz);

% Mostrar la función de transferen  cia ajustada
sys_closed

% Simular la salida del modelo ajustado con los datos de entrada
h_estimada = lsim(sys_closed, u, (0:Ts:(length(u)-1)*Ts));

% Graficar los resultados comparando la salida real y la estimada
figure;
plot((0:Ts:(length(u)-1)*Ts), h, 'b', 'LineWidth', 2); hold on;
plot((0:Ts:(length(u)-1)*Ts), h_estimada, 'r--', 'LineWidth', 2);
legend('Altura medida', 'Altura estimada por el modelo');
xlabel('Tiempo (s)');
ylabel('Altura (m)');
title('Validación del modelo ajustado');
grid on;

% Con esta función de transferencia puedo extraer la tf de la planta como
% tal del dron para luego incorporar el controlador PID y hacer una
% simulación de control PID de altura.

%% Obtención de la planta
Kp = 4.5;  % Ganancia proporcional
Ki = 2.0;  % Ganancia integral
Kd = 0.0;  % Ganancia derivativa (en este caso es 0)

% Crear el controlador PID
C = pid(Kp, Ki, Kd);

sys_plant = sys_closed / C;

%% Controlador PID de simulación
Kp_new = 2.50;
Ki_new = 1.50;
Kd_new = 0.00;

% Crear el nuevo controlador PID
C_new = pid(Kp_new, Ki_new, Kd_new);

% Sistema en lazo cerrado con el nuevo controlador
sys_closed_new = feedback(C_new * sys_plant, 1);

% Simular la respuesta al escalón
t = 0:0.01:10;  % Tiempo de simulación de 0 a 10 segundos
[y, t] = step(sys_closed_new, t);

% Simular el sistema con el PID original
C_old = pid(4.5, 2.0, 0);  % Parámetros originales del PID
sys_closed_old = feedback(C_old * sys_plant, 1);
[y_old, t] = step(sys_closed_old, t);

% Graficar ambas respuestas
figure;
plot(t, y, 'r', 'DisplayName', 'Nuevo PID');
hold on;
plot(t, y_old, 'b', 'DisplayName', 'PID Original');
title('Comparación de respuestas al escalón');
xlabel('Tiempo (s)');
ylabel('Salida');
legend;
grid on;
