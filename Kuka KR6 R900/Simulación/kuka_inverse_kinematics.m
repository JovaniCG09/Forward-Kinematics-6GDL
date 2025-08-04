clear; clc;

%% Parámetros corregidos del robot KUKA KR6 R900 (DH)
% [d, a, alpha, theta_offset]
dh_params = [
    400,  25,  -pi/2,  0;      % Joint 1
    0,    455,  0,     -pi/2;   % Joint 2 (theta_offset = -90°)
    0,    35,   pi/2,   0;      % Joint 3
    420,   0,  -pi/2,   0;      % Joint 4
    0,     0,   pi/2,   0;      % Joint 5
    80,    0,    0,     0;      % Joint 6
];

%% --- Posición deseada del efector final (mm) ---
P_des = [300; 0; 100];  % x, y, z

%% --- Orientación deseada (matriz identidad = sin rotación) ---
R_des = eye(3);

%% --- Cálculo del centro de muñeca (corregido) ---
d6 = dh_params(6,1);  % d6 = 80 mm
wrist_center = P_des - R_des * [0; 0; d6];
wx = wrist_center(1); 
wy = wrist_center(2);
wz = wrist_center(3);

%% --- Solución para q1 (corregido) ---
if abs(wx) < 1e-6 && abs(wy) < 1e-6
    q1 = 0;  % Solución arbitraria en caso singular
else
    q1 = atan2(wy, wx);
end

%% --- Solución para q2 y q3 (corregido) ---
r = sqrt(wx^2 + wy^2);
s = wz - dh_params(1,1);  % Considerar d1

% Parámetros del brazo
a2 = dh_params(2,2);  % 455 mm
a3 = dh_params(3,2);  % 35 mm

% Cálculo de D (verificado)
D = (r^2 + s^2 - a2^2 - a3^2) / (2 * a2 * a3);
D = max(min(D, 1), -1);  % Forzar a [-1,1]

% Solución codo arriba
q3 = atan2(-sqrt(1 - D^2), D);  % Solución negativa para codo arriba

% Cálculo de q2
q2 = atan2(s, r) - atan2(a3*sin(q3), a2 + a3*cos(q3));

%% --- Cálculo de R03 (corregido) ---
% Construcción de matrices DH
T01 = dh_matrix(dh_params(1,1), dh_params(1,2), dh_params(1,3), q1 + dh_params(1,4));
T12 = dh_matrix(dh_params(2,1), dh_params(2,2), dh_params(2,3), q2 + dh_params(2,4));
T23 = dh_matrix(dh_params(3,1), dh_params(3,2), dh_params(3,3), q3 + dh_params(3,4));

T03 = T01 * T12 * T23;
R03 = T03(1:3, 1:3);

%% --- Solución para orientación (q4, q5, q6) ---
R36 = R03' * R_des;

% Manejo mejorado de singularidades
r33 = R36(3,3);
if abs(r33 - 1) < 1e-6
    % Singularidad (q5 = 0)
    q5 = 0;
    q4 = 0;  % Arbitrario
    q6 = atan2(R36(2,1), R36(1,1));
elseif abs(r33 + 1) < 1e-6
    % Singularidad (q5 = π)
    q5 = pi;
    q4 = 0;  % Arbitrario
    q6 = atan2(-R36(2,1), -R36(1,1));
else
    % Solución general
    q5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), r33);
    q4 = atan2(R36(2,3), R36(1,3));
    q6 = atan2(R36(3,2), -R36(3,1));
end

thetas = [q1, q2, q3, q4, q5, q6];

%% --- Mostrar resultados ---
fprintf('\n--- Valores de las juntas (radianes) ---\n');
for i = 1:6
    fprintf('θ%d = %.4f rad (%.2f°)\n', i, thetas(i), rad2deg(thetas(i)));
end

%% --- Validación con cinemática directa ---
T_total = eye(4);
for i = 1:6
    theta = thetas(i) + dh_params(i,4);
    T_i = dh_matrix(dh_params(i,1), dh_params(i,2), dh_params(i,3), theta);
    T_total = T_total * T_i;
end

pos_alc = T_total(1:3, 4);
error_pos = norm(P_des - pos_alc);
error_rot = norm(R_des - T_total(1:3,1:3), 'fro');

fprintf('\n--- Validación ---\n');
fprintf('Posición deseada: [%.2f, %.2f, %.2f] mm\n', P_des);
fprintf('Posición alcanzada: [%.2f, %.2f, %.2f] mm\n', pos_alc);
fprintf('Error posición: %.6f mm\n', error_pos);
fprintf('Error orientación: %.6f\n', error_rot);

%% --- Generación de trayectoria para Simscape ---
% Tiempo de simulación: 5 segundos
t = linspace(0, 5, 100)';

% Configuración HOME (q2 = -90°, q3 = 90° en radianes)
q_home = [0, -pi/2, pi/2, 0, 0, 0];

% Interpolación suave (trapezoidal)
traj = zeros(100, 7);
traj(:,1) = t;
for i = 1:6
    traj(:,i+1) = smooth_trajectory(q_home(i), thetas(i), 5, 100);
end

% Asignar a workspace para Simulink
assignin('base', 'joint_trajectory', traj);

%% --- Ejecutar simulación en Simscape ---
fprintf('\nIniciando simulación en Simscape...\n');
sim('SIM_IK_KUKA_KR6_R900.slx');
fprintf('Simulación completada.\n');

%% --- Funciones auxiliares ---
function T = dh_matrix(d, a, alpha, theta)
    % Matriz de transformación DH
    T = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha)   a*sin(theta);
         0           sin(alpha)             cos(alpha)             d;
         0           0                       0                      1];
end

function q_traj = smooth_trajectory(q0, qf, t_final, n_points)
    % Genera trayectoria trapezoidal suavizada
    t = linspace(0, t_final, n_points)';
    
    % Parámetros del perfil trapezoidal
    t_acc = t_final/4;
    t_dec = 3*t_final/4;
    delta_q = qf - q0;
    
    % Velocidad máxima (calculada para perfil trapezoidal)
    v_max = 2*delta_q/(t_final + (t_acc - t_dec));
    
    % Generar perfil de posición
    q_traj = zeros(size(t));
    for i = 1:length(t)
        if t(i) <= t_acc
            % Fase de aceleración
            q_traj(i) = q0 + 0.5*v_max/t_acc * t(i)^2;
        elseif t(i) <= t_dec
            % Fase de velocidad constante
            q_traj(i) = q0 + v_max*(t(i) - t_acc/2);
        else
            % Fase de desaceleración
            q_traj(i) = qf - 0.5*v_max/(t_final-t_dec) * (t_final - t(i))^2;
        end
    end
end