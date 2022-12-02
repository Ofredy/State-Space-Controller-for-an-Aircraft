% Lateral aerodynamic coefficients
Y_beta_over_V = -1;
Y_p_over_V = 0.75;
Y_r_over_V = -0.50;
g = 9.81;
g_over_V = 0.05;
Y_A_over_V = 0.5;
Y_R_over_V = -1;
L_beta = 0.1;
L_p = 0.1;
L_r = 0.1;
L_A = 0.25;
L_R = 1;
N_beta = 0.25;
N_p = 0.25;
N_r = -0.25;
N_A = 0.25;
N_R = 0.75;

% Longitudinal aerodynamic coefficients
X_u = -0.1;
g = 9.81;
Z_u_over_V = -0.1;
Z_a_over_V = -0.1;
M_q = -0.5;
M_u = -0.05;
M_lambda = -5;
X_E = -1;
M_E = -10;
X_lambda = -14;
Z_E_over_V = -0.1;
Z_lambda_over_V = -1;


% State variables, this is an LTI system
A = [X_u, X_lambda, 0, -g, 0, 0, 0, 0, 0;
     Z_u_over_V, Z_lambda_over_V, 1, 0, 0, 0, 0, 0, 0;
     M_u, M_lambda, M_q, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, Y_beta_over_V, Y_p_over_V, (Y_r_over_V-1), g_over_V, 0;
     0, 0, 0, 0, L_beta, L_p, L_r, 0, 0;
     0, 0, 0, 0, N_beta, N_p, N_r, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0];

B = [1, 0, 0;
     1, 0, 0;
     M_E, 0, 0;
     0, 0, 0;
     0, Y_A_over_V, Y_R_over_V;
     0, L_A, L_R;
     0, N_A, N_R;
     0, 0, 0;
     0, 0, 0];

C = [0, 0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 1];

D = 0;


%%%%%%%%%% Open loop system analysis %%%%%%%%%%
open_loop_sys = ss(A, B, C, D, 'InputName', {'Elevator', 'Aileron', 'Rubber'}, 'OutputName', {'Pitch', 'Yaw', 'Roll'});
T = tf(open_loop_sys);


%%%%%%%%%% Pitch Analysis %%%%%%%%%%
figure(1)
subplot(1, 3, 1)
impulse(T(1, 1))

subplot(1, 3, 2)
step(T(1, 1))

subplot(1, 3, 3)
bode(T(1, 1))
[gm_pitch_input_elevator, pm_pitch_input_elevator] = margin(T(1, 1));


%%%%%%%%%% Yaw Analysis %%%%%%%%%%
figure(2)
subplot(3, 2, 1)
impulse(T(2, 2))
subplot(3, 2, 2)
impulse(T(2, 3))

subplot(3, 2, 3)
step(T(2, 2))
subplot(3, 2, 4)
step(T(2, 3))

subplot(3, 2, 5)
bode(T(2, 2))
[gm_yaw_input_aileron, pm_yaw_input_aileron] = margin(T(2, 2));
subplot(3, 2, 6)
bode(T(2, 3))
[gm_yaw_input_rubber, pm_yaw_input_rubber] = margin(T(2, 3));


%%%%%%%%%% Roll Analysis %%%%%%%%%%
figure(3)
subplot(3, 2, 1)
impulse(T(3, 2))
subplot(3, 2, 2)
impulse(T(3, 3))

subplot(3, 2, 3)
step(T(3, 2))
subplot(3, 2, 4)
step(T(3, 3))

subplot(3, 2, 5)
bode(T(3, 2))
[gm_roll_input_aileron, pm_roll_input_aileron] = margin(T(3, 2));
subplot(3, 2, 6)
bode(T(3, 3))
[gm_roll_input_rubber, pm_roll_input_rubber] = margin(T(3, 3));
