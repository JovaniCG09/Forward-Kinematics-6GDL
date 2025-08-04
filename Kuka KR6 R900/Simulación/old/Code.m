clear; close; clc

% Definición del tiempo
t_max = 5;
t = linspace(0, t_max, 100)';

% Definición de coordenadas cartesianas 
theta1 = [t, linspace(0, pi/8, 100)'];
theta2 = [t, linspace(0, -pi/4, 100)'];
theta3 = [t, linspace(0, pi/2, 100)'];
theta4 = [t, linspace(0, pi/2, 100)'];
theta5 = [t, linspace(pi/2, pi/2, 100)'];

% Ejecutar simulación 
sim('SIM_IK_KUKA_KR6_R900.slx');