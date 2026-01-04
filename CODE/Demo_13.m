% Tracking del motor DC con usando el control por retroalimentacion
% de posicion y velocidad
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
% GANANCIAS DE LOS CONTROLADORES
ee = 0.6;               % Relacion de amortiguamiento
wn = 144;               % Frecuencia natural
Kv = (2*ee*wn)/km;
Kp = (wn^2/km)/Kv;
Tv = Tm;
fprintf('Tv:%.4f\n', Tm)
fprintf('Kv:%.4f\n', Kv)
fprintf('Kp:%.4f\n', Kp)

%------------------------------------------------------------%
% 2. CONDICIONES DE SIMULACION
%------------------------------------------------------------%
Ts = 0.001;       % Tiempo de muestreo - VALOR PEQUEÑO
td = 2.0;          % Duracion de la simulacion
MM = ceil(td/Ts);   % Numero de pasos de simulacion
thm=1*pi/2; wm=0;

%------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%------------------------------------------------------------%
TH = zeros(MM,1);       % Para guardar "TH"
THREF = zeros(MM,1);       % Para guardar "th"
U  = zeros(MM,1);       % Para guardar "u"
ERR = zeros(MM,1);
sum_ew = 0;
for k=1:MM
    %-----------------------------------------------------%
    % SEÑALES DE REFERENCIA
    %-----------------------------------------------------%
    % SEÑAL DE REFERENCIA"qd"
    A1 = 0.5;
    thref = A1*sin(2*pi*k*Ts);     % A1*sin(pi*t)
    THREF(k) = thref;
    % PRIMERA DERIVADA DE LA REFERENCIA "dqd"
    dthref = (2*pi)*A1*cos(2*pi*k*Ts);
    % SEGUNDA DERIVADA DE LA REFERENCIA "ddqd"
    ddthref = -(2*pi)^2*A1*sin(2*pi*k*Ts);
    % TORQUE DE CARGA
    TL = 2*0.01;

    %-----------------------------------------------------%
    % CONTROLADORES
    %-----------------------------------------------------%
    % LAZO EXTERNO
    e = thref - thm;
    ERR(k) = e;
    wref = Kp*e + dthref;
    % LAZO INTERNO
    ew = wref - wm + 1*ddthref/(Kv*km); 
    sum_ew = sum_ew + ew;
    u = (Kv*Tv)*ew + Kv*Ts*sum_ew;
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
axis([0 time(end) -0.005 0.005])