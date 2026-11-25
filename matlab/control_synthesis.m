%% Self-Stabilizing Inverted Pendulum (Segway) - Control & Observer Synthesis
% Linearization around unstable vertical equilibrium, singular perturbation
% reduction of motor electrical dynamics, pole placement vs. LQR state-feedback
% synthesis, and 3-state Luenberger observer design.

clear; clc; format short g;

%% 1. Nominal Physical Parameters (used for Simulink & control synthesis)
m_b    = 0.917;      % Chassis mass [kg]
Iw_y   = 1.192e-4;   % Single wheel pitch inertia [kg.m^2]
l      = 0.186;      % Wheel track width [m]
d      = 0.055;      % Distance from wheel axle A to chassis CoM C [m]
d_i    = 0.180;      % Vertical distance from wheel axle to IMU [m]
R      = 2.2;        % Motor armature resistance [Ohm]
k_t    = 0.35;       % Motor torque / back-EMF constant [N.m/A]
rho    = 0.04;       % Wheel radius [m]
M      = 0.937;      % Total robot mass (chassis + 2 wheels) [kg]
I_psi  = 0.00250;    % Effective yaw inertia [kg.m^2]
Itot_y = 3.814e-2;   % Total pitch inertia [kg.m^2]
g      = 9.81;       % Gravitational acceleration [m/s^2]

% Reference CAD-derived parameters (for sensitivity / sim-to-real comparison):
% m_w = 0.027252; m_b = 1.160089; d = 0.06968; d_i = 0.176974;
% I_y = 1.221e-3; Iw_y = 9.557277e-6; l = 0.234; k_t = 0.38;

%% 2. Linearized Axial Dynamics: x = [p; u; theta; omega], U = U_l + U_r
N = [M + 2 * Iw_y / rho^2,  0,  m_b * d;
     0,                     1,  0;
     m_b * d,               0,  Itot_y + m_b * d^2];

A_a = [-2 * k_t^2 / (R * rho^2),  0,            2 * k_t^2 / (rho * R);
       0,                         0,            1;
       2 * k_t^2 / (rho * R),     m_b * d * g, -2 * k_t^2 / R];

B_b = [k_t / (rho * R);
       0;
       -k_t / R];

A = [0, 1, 0, 0;
     zeros(3, 1), N \ A_a];
B = [0; N \ B_b];

fprintf('--- Open-Loop Analysis ---\n');
open_loop_poles = eig(A);
disp('Open-loop eigenvalues:');
disp(open_loop_poles.');
fprintf('Controllability matrix rank: %d / %d\n\n', rank(ctrb(A, B)), size(A, 1));

%% 3. State-Feedback Controller: Pole Placement
% Three-timescale cascade preserving fast natural actuator mode
zeta    = 0.7;
omega_n = 3.0 / zeta;

lambda_fast = min(real(open_loop_poles));
lambda_slow = -zeta * omega_n / 100.0;
lambda_c1   = -zeta * omega_n + 1i * omega_n * sqrt(1 - zeta^2);
lambda_c2   = conj(lambda_c1);

desired_poles = [lambda_slow, lambda_c1, lambda_c2, lambda_fast];
K_place = place(A, B, desired_poles);

fprintf('--- Pole Placement Controller ---\n');
disp('K_place:');
disp(K_place);
disp('Closed-loop poles (pole placement):');
disp(eig(A - B * K_place).');

%% 4. 3-State Luenberger Observer: z = [p; xi; theta], y = (alpha_l + alpha_r) / 2
A_obs = [0, 1, 0;
         0, 0, g;
         0, 0, 0];
B_obs = [-d_i, 0;
         0,    1;
         1,    0];
C_obs = [1 / rho, 0, -1];

fprintf('--- Luenberger Observer ---\n');
fprintf('Observability matrix rank: %d / %d\n', rank(obsv(A_obs, C_obs)), size(A_obs, 1));

observer_poles = [-15, -9, -6];
L_obs = place(A_obs', C_obs', observer_poles)';

disp('L_obs (pole placement):');
disp(L_obs.');
disp('Observer error eigenvalues:');
disp(eig(A_obs - L_obs * C_obs).');

%% 5. LQR Controller & Dual Observer
K_lqr = lqr(A, B, eye(4), 1);
L_lqr = lqr(A_obs', C_obs', eye(3), 1)';

fprintf('--- LQR Synthesis ---\n');
disp('K_lqr:');
disp(K_lqr);
disp('Closed-loop poles (LQR):');
disp(eig(A - B * K_lqr).');
disp('L_lqr:');
disp(L_lqr.');
disp('Observer eigenvalues (LQR):');
disp(eig(A_obs - L_lqr * C_obs).');

%% 6. Empirically Tuned Gains Deployed on Hardware (Arduino)
K_hw = [-1.5, -80.0, -80.482, -17.112];
L_hw = [1.3; 11.2; 3.3];

fprintf('--- Hardware-Validated Gains ---\n');
disp('Closed-loop poles (K_hw on nominal model):');
disp(eig(A - B * K_hw).');
disp('Observer eigenvalues (L_hw):');
disp(eig(A_obs - L_hw * C_obs).');