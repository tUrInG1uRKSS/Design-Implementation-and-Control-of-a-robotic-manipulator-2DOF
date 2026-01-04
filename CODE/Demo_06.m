% Control de posicion del motor DC con retroalimentacion de 
% posicion y velocidad usando ecuaciones diferenciales
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
td = 0.2;          % Duracion de la simulacion
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
    thref = 1;
    TL = 5*0.01;
    
    %-----------------------------------------------------%
    % CONTROLADORES
    %-----------------------------------------------------%
    % LAZO EXTERNO
    e = thref - thm;
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
plot(time, TH);
xlabel('[s]');
ylabel('[rad]');
title('Posicion');
grid on
% ERROR
subplot(3,1,3)
plot(time, ERR);
xlabel('[s]');
ylabel('[rad]');
title('Error');
grid on