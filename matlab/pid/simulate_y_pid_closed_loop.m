clear; clc; close all;

%% =========================================================
%  USER SETTINGS
% ==========================================================
csv_file = ('/home/sejunmoon/self_driving_wheelchair/log/cmd_vel_lilnear_velocity_log(y_only).csv');

% From your ESP32 controller settings
y_neutral = 2.689;
y_span    = 0.80;

% Train / validation split
train_ratio = 0.7;

% If you want mild smoothing on v_body for identification
use_smoothing = false;
smooth_window = 5;   % moving average window

%% =========================================================
%  LOAD CSV
% ==========================================================
T = readtable(csv_file);

disp('CSV columns:');
disp(T.Properties.VariableNames);

required_cols = {'time_ms','ref_v','ref_w','y_voltage','x_voltage','vL','vR','v','w'};
for i = 1:numel(required_cols)
    if ~ismember(required_cols{i}, T.Properties.VariableNames)
        error('Missing required column: %s', required_cols{i});
    end
end

%% =========================================================
%  EXTRACT SIGNALS
% ==========================================================
time_ms   = T.time_ms;
time_s    = (time_ms - time_ms(1)) / 1000.0;

ref_v     = T.ref_v;
ref_w     = T.ref_w;
y_voltage = T.y_voltage;
x_voltage = T.x_voltage;
vL        = T.vL;
vR        = T.vR;
v_body    = T.v;
w_body    = T.w;

%% =========================================================
%  BASIC TIME CHECK
% ==========================================================
dt = diff(time_s);

if any(dt <= 0)
    error('Time is not strictly increasing.');
end

Ts_mean = mean(dt);
Ts_std  = std(dt);
Fs_mean = 1 / Ts_mean;

fprintf('\n===== Time Info =====\n');
fprintf('Samples             : %d\n', length(time_s));
fprintf('Mean Ts             : %.6f s\n', Ts_mean);
fprintf('Std of Ts           : %.6f s\n', Ts_std);
fprintf('Mean Fs             : %.3f Hz\n', Fs_mean);
fprintf('Duration            : %.3f s\n', time_s(end) - time_s(1));

%% =========================================================
%  NORMALIZE INPUT
% ==========================================================
u_y = (y_voltage - y_neutral) / y_span;
u_y = max(min(u_y, 1.0), -1.0);

% Optional smoothing
v_body_used = v_body;
if use_smoothing
    v_body_used = movmean(v_body_used, smooth_window);
end

%% =========================================================
%  QUICK SANITY PRINTS
% ==========================================================
fprintf('\n===== Signal Ranges =====\n');
fprintf('ref_v      : [%.4f, %.4f]\n', min(ref_v), max(ref_v));
fprintf('ref_w      : [%.4f, %.4f]\n', min(ref_w), max(ref_w));
fprintf('y_voltage  : [%.4f, %.4f]\n', min(y_voltage), max(y_voltage));
fprintf('x_voltage  : [%.4f, %.4f]\n', min(x_voltage), max(x_voltage));
fprintf('u_y        : [%.4f, %.4f]\n', min(u_y), max(u_y));
fprintf('vL         : [%.4f, %.4f]\n', min(vL), max(vL));
fprintf('vR         : [%.4f, %.4f]\n', min(vR), max(vR));
fprintf('v_body     : [%.4f, %.4f]\n', min(v_body), max(v_body));
fprintf('w_body     : [%.4f, %.4f]\n', min(w_body), max(w_body));

%% =========================================================
%  RAW DATA PLOTS
% ==========================================================
figure('Name','Raw Signals Overview','Color','w');

subplot(4,1,1);
plot(time_s, ref_v, 'LineWidth', 1.2); hold on;
plot(time_s, ref_w, 'LineWidth', 1.2);
grid on;
ylabel('ref');
legend('ref\_v','ref\_w','Location','best');
title('Reference commands');

subplot(4,1,2);
plot(time_s, y_voltage, 'LineWidth', 1.2); hold on;
plot(time_s, x_voltage, 'LineWidth', 1.2);
grid on;
ylabel('voltage (V)');
legend('y voltage','x voltage','Location','best');
title('Actual actuator voltage');

subplot(4,1,3);
plot(time_s, u_y, 'LineWidth', 1.2);
grid on;
ylabel('u_y norm');
title('Normalized input from y\_voltage');

subplot(4,1,4);
plot(time_s, v_body, 'LineWidth', 1.2); hold on;
plot(time_s, w_body, 'LineWidth', 1.2);
grid on;
ylabel('body states');
xlabel('Time (s)');
legend('v body','w body','Location','best');
title('Body motion');

%% =========================================================
%  WHEEL AND BODY CHECK
% ==========================================================
figure('Name','Wheel / Body Behavior','Color','w');

subplot(3,1,1);
plot(time_s, vL, 'LineWidth', 1.2); hold on;
plot(time_s, vR, 'LineWidth', 1.2);
grid on;
ylabel('wheel v (m/s)');
legend('vL','vR','Location','best');
title('Left / Right wheel velocity');

subplot(3,1,2);
plot(time_s, v_body, 'LineWidth', 1.2);
grid on;
ylabel('v body (m/s)');
title('Body linear velocity');

subplot(3,1,3);
plot(time_s, w_body, 'LineWidth', 1.2);
grid on;
ylabel('w body (rad/s)');
xlabel('Time (s)');
title('Body angular velocity');

%% =========================================================
%  STRAIGHT DRIVING BIAS CHECK
% ==========================================================
% Since this is y-only data, check if turning still appears
wheel_diff = vR - vL;

figure('Name','Straight Driving Bias Check','Color','w');

subplot(3,1,1);
plot(time_s, u_y, 'LineWidth', 1.2);
grid on;
ylabel('u_y');
title('Input');

subplot(3,1,2);
plot(time_s, wheel_diff, 'LineWidth', 1.2);
grid on;
ylabel('vR - vL');
title('Wheel mismatch');

subplot(3,1,3);
plot(time_s, w_body, 'LineWidth', 1.2);
grid on;
ylabel('w body');
xlabel('Time (s)');
title('Turning generated during y-only test');

fprintf('\n===== Straight Driving Bias Info =====\n');
fprintf('Mean(vR - vL)  = %.6f m/s\n', mean(wheel_diff));
fprintf('Mean(|vR-vL|)  = %.6f m/s\n', mean(abs(wheel_diff)));
fprintf('Mean(w_body)   = %.6f rad/s\n', mean(w_body));
fprintf('Mean(|w_body|) = %.6f rad/s\n', mean(abs(w_body)));

%% =========================================================
%  BUILD IDENTIFICATION DATA
% ==========================================================
% Main target: u_y -> v_body
z_uv = iddata(v_body_used, u_y, Ts_mean);

% Extra comparisons
z_yv = iddata(v_body_used, y_voltage, Ts_mean);
z_rv = iddata(v_body_used, ref_v, Ts_mean);

%% =========================================================
%  TRAIN / VALIDATION SPLIT
% ==========================================================
N = length(v_body_used);
Ntr = round(train_ratio * N);

if Ntr < 20 || (N - Ntr) < 20
    error('Not enough data after split.');
end

z_uv_train = z_uv(1:Ntr);
z_uv_val   = z_uv(Ntr+1:end);

z_yv_train = z_yv(1:Ntr);
z_yv_val   = z_yv(Ntr+1:end);

z_rv_train = z_rv(1:Ntr);
z_rv_val   = z_rv(Ntr+1:end);

fprintf('\n===== Data Split =====\n');
fprintf('Train samples      : %d\n', Ntr);
fprintf('Validation samples : %d\n', N - Ntr);

%% =========================================================
%  ESTIMATE MODELS: u_y -> v_body
% ==========================================================
disp('Estimating models for u_y -> v_body ...');

sys_uv_tf1 = tfest(z_uv_train, 1);
sys_uv_tf2 = tfest(z_uv_train, 2);
sys_uv_ss2 = ssest(z_uv_train, 2);
sys_uv_ss3 = ssest(z_uv_train, 3);

figure('Name','Compare: u_y -> v_body','Color','w');
compare(z_uv_val, sys_uv_tf1, sys_uv_tf2, sys_uv_ss2, sys_uv_ss3);
grid on;
title('Validation compare: u_y -> v_body');

%% =========================================================
%  ESTIMATE MODELS: y_voltage -> v_body
% ==========================================================
disp('Estimating models for y_voltage -> v_body ...');

sys_yv_tf1 = tfest(z_yv_train, 1);
sys_yv_tf2 = tfest(z_yv_train, 2);
sys_yv_ss2 = ssest(z_yv_train, 2);

figure('Name','Compare: y_voltage -> v_body','Color','w');
compare(z_yv_val, sys_yv_tf1, sys_yv_tf2, sys_yv_ss2);
grid on;
title('Validation compare: y_voltage -> v_body');

%% =========================================================
%  ESTIMATE MODELS: ref_v -> v_body (reference only)
% ==========================================================
disp('Estimating models for ref_v -> v_body ...');

sys_rv_tf1 = tfest(z_rv_train, 1);
sys_rv_tf2 = tfest(z_rv_train, 2);

figure('Name','Compare: ref_v -> v_body','Color','w');
compare(z_rv_val, sys_rv_tf1, sys_rv_tf2);
grid on;
title('Validation compare: ref_v -> v_body');

%% =========================================================
%  STEP RESPONSE CHECK
% ==========================================================
figure('Name','Step Responses','Color','w');

subplot(2,2,1);
step(sys_uv_tf1); grid on; title('u_y -> v_body | tf1');

subplot(2,2,2);
step(sys_uv_tf2); grid on; title('u_y -> v_body | tf2');

subplot(2,2,3);
step(sys_uv_ss2); grid on; title('u_y -> v_body | ss2');

subplot(2,2,4);
step(sys_uv_ss3); grid on; title('u_y -> v_body | ss3');

%% =========================================================
%  POLE / STABILITY CHECK
% ==========================================================
disp(' ');
disp('===== Pole Check: u_y -> v_body =====');
disp('sys_uv_tf1 poles:'); disp(pole(sys_uv_tf1));
disp('sys_uv_tf2 poles:'); disp(pole(sys_uv_tf2));
disp('sys_uv_ss2 poles:'); disp(pole(sys_uv_ss2));
disp('sys_uv_ss3 poles:'); disp(pole(sys_uv_ss3));

%% =========================================================
%  MODEL SUMMARY
% ==========================================================
disp(' ');
disp('===== Model Summary =====');

disp('--- u_y -> v_body ---');
sys_uv_tf1
sys_uv_tf2
sys_uv_ss2
sys_uv_ss3

disp('--- y_voltage -> v_body ---');
sys_yv_tf1
sys_yv_tf2
sys_yv_ss2

disp('--- ref_v -> v_body ---');
sys_rv_tf1
sys_rv_tf2

%% =========================================================
%  OPTIONAL: SAVE BEST CANDIDATES
% ==========================================================
save('identified_forward_models.mat', ...
    'sys_uv_tf1','sys_uv_tf2','sys_uv_ss2','sys_uv_ss3', ...
    'sys_yv_tf1','sys_yv_tf2','sys_yv_ss2', ...
    'sys_rv_tf1','sys_rv_tf2', ...
    'Ts_mean');

fprintf('\nSaved identified models to identified_forward_models.mat\n');