% CONTROL OPTIMO DE VELOCIDAD DE UN MOTORDC
clc; clearvars; close all;

%--------------------------------------------------------------%
% 1. DATOS DEL SISTEMA DEL MOTOR
%--------------------------------------------------------------%
% CARGAMOS LOS PARAMETROS DEL MOTOR
parametros_motor_GM9236S025
% ECUACION DE ESPACIO ESTADO
A = [-Bm/Jm  Kt/Jm
     -Kb/La  -Ra/La]; 
B = [0;1/La];
Cc = [1 0];   
Dc = 0;
sys = ss(A,B,Cc,Dc);
% CHEQUEAR CONTROLABILIDAD Y OBSERVABILIDAD
M = [B A*B]; 
N = [Cc' A'*Cc']; 
% rank(M)=rank(N)=n=2 => c.c y c.o.

% DISCRETIZACION DEL SISTEMA
Ts = 0.001;
[G,H,C,D] = c2dm(A,B,Cc,Dc,Ts,'zoh');

% DISEÑO DEL REGULADOR LINEAL CUADRATICO PARA EL SISTEMA DISCRETO
Q = [0.001   0;
       0   0.001]; 
R = [0.1]; 
[K,P,E] = dlqr(G,H,Q,R); 
k1 = K(1);  % GANANCIA OPTIMA K

% CORRECCION DE r(k)
g = 1/(C*inv(eye(2)-G+H*K)*H*k1);

%--------------------------------------------------------------%
% 3. LAZO DE SIMULACION
%--------------------------------------------------------------%
td = 0.2;           % Duracion de la simulacion
MM = ceil(td/Ts);   % Numero de pasos de simulacion
V = zeros(MM,1); 
Y = zeros(MM,1);
x = [0;0]; 
for k=1:MM
    %-----------------------------------------------------%
    % REFERENCIA
    %-----------------------------------------------------%
    wref = 100;
    r = wref*g;
    
    %-----------------------------------------------------%
    % SEÑAL DE CONTROL
    %-----------------------------------------------------%
    v = -K*x + k1*r;
    if(abs(v)>12)
        v = sign(v)*12;
    end
    V(k) = v;
    
    %-----------------------------------------------------%
    % RESPUESTA DEL SISTEMA
    %-----------------------------------------------------%
    x = G*x + H*v;
    Y(k) = x(1);
end


%------------------------------------------------------------%
% 4. PLOTEO DE RESULTADOS
%------------------------------------------------------------%
% EJE DE TIEMPO
time = linspace(0,MM*Ts,MM);
figure
% VELOCIDAD
subplot(2,1,1)
plot(time,Y);
ylabel('y(rad/s)'); 
grid on
% SEÑAL DE CONTROL
subplot(2,1,2)
plot(time,V); 
ylabel('V(voltios)'); 
grid on
xlabel('Tiempo(s)')
  

% CALCULO RECURSIVO DE LA MATRIZ Pr Y CALCULO DE Kr
T = 15; Pr = zeros(2,2);
for i=1:T
    Pr = Q + G'*Pr*G - G'*Pr*H*inv(R+H'*Pr*H)*H'*Pr*G;
end
Kr = inv(R + H'*P*H)*H'*P*G;
% SE CUMPLE QUE K=Kr Y P=Pr
