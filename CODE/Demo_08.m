% Control de posicion en el eje de salida del motor DC con 
% retroalimentacion de posicion y velocidad usando 
% ecuaciones diferenciales
clc; clear all; close all

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
ee = 0.9;               % Relacion de amortiguamiento
wn = 10;               % Frecuencia natural
Kv = (2*ee*wn)/km;
Kp = (wn^2/km)/Kv;
Tv = Tm;
fprintf('Tv:%.4f\n', Tm)
fprintf('Kv:%.4f\n', Kv)
fprintf('Kp:%.4f\n', Kp)

%------------------------------------------------------------%
% 2. CONDICIONES DE SIMULACION
%------------------------------------------------------------%
Ts = 0.02;       % Tiempo de muestreo - VALOR PEQUEÑO
td = 1;          % Duracion de la simulacion
MM = ceil(td/Ts);   % Numero de pasos de simulacion
thm=0; wm=0;

%------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%------------------------------------------------------------%
TH = zeros(MM,1);       % Para guardar "w"
U  = zeros(MM,1);       % Para guardar "u"
ERR = zeros(MM,1);
sum_ew = 0;
for k=1:MM
    %-----------------------------------------------------%
    % SEÑALES DE REFERENCIA
    %-----------------------------------------------------%
    qref = 1;
    TL = 0*0.01;
    
    %-----------------------------------------------------%
    % CONTROLADORES
    %-----------------------------------------------------%
    % LECTURA DEL ENCODER
    q = thm/n;
    % LAZO EXTERNO
    e = qref*n - q*n;
    ERR(k) = e;
    wref = Kp*e;
    % LAZO INTERNO
    ew = wref - wm; 
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
%  5.1. EJE DE TIEMPO
time = linspace(0,MM*Ts,MM);
% CREAMOS LA FIGURA
figure('position', [100, 100, 800, 700])
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
plot(time, TH/n);  % Convertimos al eje de salida
xlabel('[s]');
ylabel('[rad]');
title('Posicion');
grid on
axis([0 time(end) 0 1.5])
% ERROR
subplot(3,1,3)
plot(time, ERR/n);
xlabel('[s]');
ylabel('[rad]');
title('Error');
grid on