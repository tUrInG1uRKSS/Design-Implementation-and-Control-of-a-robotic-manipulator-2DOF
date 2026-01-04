% Control de posicion del motor DC con retroalimentacion de
% posicion usando ecuaciones diferenciales
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
td = 0.5;          % Duracion de la simulacion
MM = ceil(td/Ts);   % Numero de pasos de simulacion
thm=0; wm=0;

%------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%------------------------------------------------------------%
TH = zeros(MM,1);       % Para guardar "w"
U  = zeros(MM,1);       % Para guardar "u"
ERR = zeros(MM,1);      
sum_e = 0;
for k=1:MM
    %-----------------------------------------------------%
    % SEÑALES DE REFERENCIA
    %-----------------------------------------------------%
    thref = 1;
    TL = 0*0.01;
    
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
axis([0 time(end) 0 1.5])
% ERROR
subplot(3,1,3)
plot(time, ERR);
xlabel('[s]');
ylabel('[rad]');
title('Error');
grid on