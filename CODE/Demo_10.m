% Tracking del motor DC con usando el control por retroalimentacion
% de posicion
clc; clear all; close all

%--------------------------------------------------------------%
% 1. DATOS DEL SISTEMA DEL MOTOR
%--------------------------------------------------------------%
% CARGAMOS LOS PARAMETROS DEL MOTOR
parametros_motor_GM9236S025
% PARAMETROS DEL AMPLIFICADOR
Gv = 12/3.3;
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

%------------------------------------------------------------%
% 2. CONDICIONES DE SIMULACION
%------------------------------------------------------------%
Ts = 0.001;       % Tiempo de muestreo
td = 2.0;          % Duracion de la simulacion
MM = ceil(td/Ts);   % Numero de pasos de simulacion
thm=1*pi/2; wm=0;

%------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%------------------------------------------------------------%
TH = zeros(MM,1);       % Para guardar "th"
THREF = zeros(MM,1);       % Para guardar "th"
U  = zeros(MM,1);       % Para guardar "u"
ERR = zeros(MM,1);      
sum_e = 0;
for k=1:MM
    %-----------------------------------------------------%
    % SEÑALES DE REFERENCIA
    %-----------------------------------------------------%
    % SEÑAL DE REFERENCIA"qd"
    A1 = 0.5;
    thref = A1*sin(2*pi*k*Ts);     % A1*sin(2*pi*t)
    THREF(k) = thref;
    % TORQUE DE CARGA
    TL = 2*0.01;
    
    %-----------------------------------------------------%
    % CONTROLADOR PI
    %-----------------------------------------------------%
    %  SEÑAL DE ERROR
    e     = thref - thm;
    sum_e = sum_e + e;
    ERR(k) = e;
    %  SEÑAL DE CONTROL
    u = (Kp*Tp)*e + Kp*Ts*sum_e;
    if(abs(u)>3.3)
        u = sign(u)*3.3;
    end
    U(k) = u;   % sale del arduino
    %  VOLTAJE DE ARMADURA
    Va = Gv*u;
    
    %-----------------------------------------------------%
    % RESPUESTA DEL SISTEMA
    %-----------------------------------------------------%
    % HALLAMOS LA ACELERACION
    dwm = (Kt/Ra)*Va - (Bm+Kt*Kb/Ra)*wm - TL;
    dwm = dwm/Jm;
    % CALCULAMOS LA NUEVA VELOCIDAD
    wm  = wm + Ts*dwm;
    thm = thm + Ts*wm;
    TH(k+1) = thm;
end
TH(end) = [];


%------------------------------------------------------------%
% 4. PLOTEO DE RESULTADOS
%------------------------------------------------------------%
% EJE DE TIEMPO
time = linspace(0,MM*Ts,MM);
% CREAMOS LA FIGURA
figure('position', [100, 100, 600, 700])
hold on
grid on
% VOLTAJE
subplot(3,1,1)
plot(time, U*Gv);
ylabel('[V]');
title('Voltaje Va');
grid on
axis([0 time(end) -15 15])
% POSICION
subplot(3,1,2)
plot(time, TH, time, THREF);
legend('th', 'th ref')
xlabel('[s]');
ylabel('[rad]');
title('Posicion');
grid on
axis([0 time(end) -1.5 1.5])
% ERROR
subplot(3,1,3)
plot(time, ERR);
xlabel('[s]');
ylabel('[rad]');
title('Error');
grid on
axis([0 time(end) -0.2 0.2])