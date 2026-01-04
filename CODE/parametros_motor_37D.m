% Parametros del motor PITMAN GM9236S025
clc; clear all; close all

%--------------------------------------------------------------%
% 1. DATOS DEL SISTEMA DEL MOTOR
%--------------------------------------------------------------%
%  1.1. SUBSISTEMA ELECTRICO    
Ra = 4.952082603;         % Resistencia de armadura
La = 0.005499614571;
Kt = 0.0055601;     % Constantes
Kb = Kt;
Jm = 9.3291e-7;       % Inercia
Bm = 6.9167e-06;      % Friccion
n=70;

% VALORES EQUIVALENTES
%Jeq = Jm;
%Beq = Bm + Kt*Kb/Ra;