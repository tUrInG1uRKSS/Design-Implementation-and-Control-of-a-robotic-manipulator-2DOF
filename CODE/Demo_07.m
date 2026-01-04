% Control de posicion en el eje de salida del motor DC con 
% retroalimentacion de posicion y velocidad 
clc; clear all

%--------------------------------------------------------------%
% 1. DATOS DEL SISTEMA DEL MOTOR
%--------------------------------------------------------------%
% CARGAMOS LOS PARAMETROS DEL MOTOR
%parametros_motor_37D
parametros_motor_25D
% PARAMETROS DEL AMPLIFICADOR
Gv = 12/5;
% PARAMETROS DEL MODELO SIMPLIFICADO DEL MOTOR
km = 1/Kb;
Tm = (Ra*Jm)/(Kb*Kt);
% GANANCIAS DE LOS CONTROLADORES
Tv = Tm;
ee = 0.9;          % Relacion de amortiguamiento
wn = 25;            % Frecuencia natural
Kv = (2*ee*wn)/km;
Kp = (wn^2/km)/Kv;
fprintf('Tv:%.4f\n', Tm)
fprintf('Kv:%.4f\n', Kv)
fprintf('Kp:%.4f\n', Kp)

%--------------------------------------------------------------%
% 2. PARAMETROS DE SIMULACION 
%--------------------------------------------------------------%
Tc = 0.001;         % Tiempo de integracion (LAZO INTERNO)
td = 0.5;           % Duracion de la simulacion
Ts = Tc;            % Tiempo de muestreo para leer resultados
% SEÑALES DE ENTRADA
th_ref = 1;         % Señal de referencia
TL = 0*0.01;     % Torque de carga

%------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%------------------------------------------------------------%
sim('Demo_07_sim.slx')

%------------------------------------------------------------%
% 4. PLOTEO DE RESULTADOS
%------------------------------------------------------------%
% CREAMOS LA FIGURA
figure('position', [100, 100, 800, 700])
hold on
grid on
% VOLTAJE
subplot(3,1,1)
plot(time, Va);
ylabel('[V]');
title('Voltaje Va');
grid on
axis([0 time(end) -15 15])
% POSICION
subplot(3,1,2)
plot(time, th);
xlabel('[s]');
ylabel('[rad]');
title('Posicion');
grid on
axis([time(1) time(end) 0 1.5])
% ERROR
subplot(3,1,3)
plot(time, err/n);
xlabel('[s]');
ylabel('[rad]');
title('Error');
grid on