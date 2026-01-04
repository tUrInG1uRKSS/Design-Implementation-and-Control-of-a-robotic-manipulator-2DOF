% Control de posicion del motor DC con retroalimentacion de
% posicion
clc; clear all

%--------------------------------------------------------------%
% 1. DATOS DEL SISTEMA DEL MOTOR
%--------------------------------------------------------------%
% CARGAMOS LOS PARAMETROS DEL MOTOR
parametros_motor_37D
% PARAMETROS DEL AMPLIFICADOR
Gv = 12/5;
% PARAMETROS DEL MODELO SIMPLIFICADO DEL MOTOR
km = 1/Kb;
Tm = (Ra*Jm)/(Kb*Kt);
% GANANCIAS DEL CONTROLADOR PI
Kp = 0.02;
Tp = 100;
fprintf('Tm:%.4f\n', Tm)
fprintf('Tp:%.4f\n', Tp)
if(Tp<Tm)
    disp('Inestable')
else
    disp('Estable')
end

%--------------------------------------------------------------%
% 2. PARAMETROS DE SIMULACION 
%--------------------------------------------------------------%
Tc = 0.001;         % Tiempo de integracion
td = 0.2;           % Duracion de la simulacion
Ts = Tc;            % Tiempo de muestreo para leer resultados
% SEÑALES DE ENTRADA
th_ref = 1;         % Señal de referencia
TL = 5*0.01;    	% Torque de carga

%------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%------------------------------------------------------------%
sim('Demo_01_sim.slx')

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
axis([0 time(end) 0 1.5])
% ERROR
subplot(3,1,3)
plot(time, err);
xlabel('[s]');
ylabel('[rad]');
title('Error');
grid on