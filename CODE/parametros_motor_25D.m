% Parametros del motor PITMAN GM9236S025
clc; clear all; close all

%--------------------------------------------------------------%
% 1. DATOS DEL SISTEMA DEL MOTOR
%--------------------------------------------------------------%
%  1.1. SUBSISTEMA ELECTRICO    
Ra = 8.751992253;         % Resistencia de armadura
La = 0.01457693666;
Kt = 0.0052226;     % Constantes
Kb = Kt;
Jm = 3.0164e-7;       % Inercia
Bm = 9.9111e-06;      % Friccion
n=78.09;

% VALORES EQUIVALENTES
%Jeq = Jm;
%Beq = Bm + Kt*Kb/Ra;