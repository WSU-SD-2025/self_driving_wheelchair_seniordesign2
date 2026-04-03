clc; clear; close all;

% ===== load models =====
S1 = load('/home/sejunmoon/self_driving_wheelchair/matlab/models/y_model_L.mat');
S2 = load('/home/sejunmoon/self_driving_wheelchair/matlab/models/y_model_R.mat');
S3 = load('/home/sejunmoon/self_driving_wheelchair/matlab/models/x_model_L.mat');
S4 = load('/home/sejunmoon/self_driving_wheelchair/matlab/models/x_model_R.mat');

fn1 = fieldnames(S1); y_model_L = S1.(fn1{1});
fn2 = fieldnames(S2); y_model_R = S2.(fn2{1});
fn3 = fieldnames(S3); x_model_L = S3.(fn3{1});
fn4 = fieldnames(S4); x_model_R = S4.(fn4{1});

% ===== csv =====
T = readtable('/home/sejunmoon/self_driving_wheelchair/log/cmd_vel_combined.csv');

t = T.time_ms / 1000;
Ts = mean(diff(t));
N = length(t);

v_ref = T.ref_v;
w_ref = T.ref_w;

% ===== constants =====
Y_NEUTRAL = 2.689;
X_NEUTRAL = 2.705;
V_MIN = 1.0;
V_MAX = 4.2;
WHEEL_SEPARATION = 0.575;

% ===== PID gains =====
% linear
Kp_y = 1.2;
Ki_y = 0.35;
Kd_y = 0.01;

% angular
Kp_x = 0.45;
Ki_x = 0.05;
Kd_x = 0.00;

% ===== init =====
int_y = 0; prev_e_y = 0;
int_x = 0; prev_e_x = 0;

v_meas = 0;
w_meas = 0;

y_hist = Y_NEUTRAL * ones(N,1);
x_hist = X_NEUTRAL * ones(N,1);

vL_log = zeros(N,1);
vR_log = zeros(N,1);
v_log  = zeros(N,1);
w_log  = zeros(N,1);
y_cmd_log = zeros(N,1);
x_cmd_log = zeros(N,1);

for k = 2:N

    % ===== Y PID =====
    e_y = v_ref(k) - v_meas;
    der_y = (e_y - prev_e_y) / Ts;

    int_y_candidate = int_y + e_y * Ts;
    y_pid_candidate = Kp_y*e_y + Ki_y*int_y_candidate + Kd_y*der_y;

    y_cmd_unsat = Y_NEUTRAL + y_pid_candidate;
    y_cmd = max(V_MIN, min(V_MAX, y_cmd_unsat));

    if y_cmd == y_cmd_unsat
        int_y = int_y_candidate;
    end

    prev_e_y = e_y;

    % ===== X PID =====
    e_x = w_ref(k) - w_meas;
    der_x = (e_x - prev_e_x) / Ts;

    int_x_candidate = int_x + e_x * Ts;
    x_pid_candidate = Kp_x*e_x + Ki_x*int_x_candidate + Kd_x*der_x;

    x_cmd_unsat = X_NEUTRAL - x_pid_candidate;
    x_cmd = max(V_MIN, min(V_MAX, x_cmd_unsat));

    if x_cmd == x_cmd_unsat
        int_x = int_x_candidate;
    end

    prev_e_x = e_x;

    % ===== save command history =====
    y_hist(k) = y_cmd;
    x_hist(k) = x_cmd;

    % ===== simulate models =====
    data_y = iddata([], y_hist(1:k), Ts);
    data_x = iddata([], x_hist(1:k), Ts);

    yL = sim(y_model_L, data_y);
    yR = sim(y_model_R, data_y);
    xL = sim(x_model_L, data_x);
    xR = sim(x_model_R, data_x);

    % combine y and x effects
    vL = yL.OutputData(end) + xL.OutputData(end);
    vR = yR.OutputData(end) + xR.OutputData(end);

    v_meas = (vL + vR) / 2;
    w_meas = (vR - vL) / WHEEL_SEPARATION;

    % ===== log =====
    vL_log(k) = vL;
    vR_log(k) = vR;
    v_log(k)  = v_meas;
    w_log(k)  = w_meas;
    y_cmd_log(k) = y_cmd;
    x_cmd_log(k) = x_cmd;
end

% ===== plots =====
figure;
plot(t, v_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, v_log, 'b', 'LineWidth', 1.5);
grid on;
legend('v_{ref}', 'v_{meas}');
title('Combined PID: Linear tracking');
xlabel('Time [s]');
ylabel('Linear velocity [m/s]');

figure;
plot(t, w_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, w_log, 'b', 'LineWidth', 1.5);
grid on;
legend('w_{ref}', 'w_{meas}');
title('Combined PID: Angular tracking');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');

figure;
plot(t, y_cmd_log, 'LineWidth', 1.3); hold on;
plot(t, x_cmd_log, 'LineWidth', 1.3);
grid on;
legend('y\_cmd', 'x\_cmd');
title('Command voltages');
xlabel('Time [s]');
ylabel('Voltage [V]');

figure;
plot(t, vL_log, 'LineWidth', 1.2); hold on;
plot(t, vR_log, 'LineWidth', 1.2);
grid on;
legend('vL', 'vR');
title('Wheel velocities');
xlabel('Time [s]');
ylabel('Velocity [m/s]');