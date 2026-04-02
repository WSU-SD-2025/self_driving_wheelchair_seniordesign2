clear; clc; close all;

%% =========================
%  Load models
% =========================
model_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/models';

load(fullfile(model_dir, 'model_y_vL.mat'));   % variable: model_y_vL
load(fullfile(model_dir, 'model_y_vR.mat'));   % variable: model_y_vR
load(fullfile(model_dir, 'model_x_vL.mat'));   % variable: model_x_vL
load(fullfile(model_dir, 'model_x_vR.mat'));   % variable: model_x_vR

Ts = model_y_vL.Ts;

if model_y_vR.Ts ~= Ts || model_x_vL.Ts ~= Ts || model_x_vR.Ts ~= Ts
    error('All model sample times must match.');
end

%% =========================
%  Parameters (match main.cpp)
% =========================
Y_NEUTRAL = 2.689;
X_NEUTRAL = 2.6951;

Y_MIN_V = 1.0;
Y_MAX_V = 4.2;
X_MIN_V = 1.0;
X_MAX_V = 4.2;

WHEEL_SEPARATION = 0.575;

Y_FF_GAIN = 1.0;
X_FF_GAIN = 0.30;

TURN_ENABLE_THRESHOLD = 0.03;
SPEED_ENABLE_THRESHOLD = 0.02;

% main.cpp initial PID values
KpY = 0.10;  KiY = 0.01;  KdY = 0.0;
KpW = 0.12;  KiW = 0.01;  KdW = 0.0;

Y_CORR_MIN = -0.20;  Y_CORR_MAX = 0.20;
X_CORR_MIN = -0.18;  X_CORR_MAX = 0.18;

trim_y = 0.0;
trim_x = 0.0;

%% =========================
%  Simulation horizon
% =========================
Tend = 20;
t = (0:Ts:Tend)';
N = numel(t);

%% =========================
%  cmd_vel scenario
% =========================
% 0~2s     stop
% 2~6s     forward
% 6~10s    forward + left turn
% 10~14s   forward + right turn
% 14~17s   stop
% 17~20s   backward

v_ref_vec = zeros(N,1);
w_ref_vec = zeros(N,1);

v_ref_vec(t >= 2  & t < 6 ) = 0.30;
v_ref_vec(t >= 6  & t < 10) = 0.30;
w_ref_vec(t >= 6  & t < 10) = 0.35;

v_ref_vec(t >= 10 & t < 14) = 0.30;
w_ref_vec(t >= 10 & t < 14) = -0.35;

v_ref_vec(t >= 17) = -0.20;

%% =========================
%  Preallocate logs
% =========================
y_ff     = zeros(N,1);
x_ff     = zeros(N,1);

y_corr_v = zeros(N,1);
x_corr_v = zeros(N,1);

y_cmd    = zeros(N,1);
x_cmd    = zeros(N,1);

vL_y = zeros(N,1);
vR_y = zeros(N,1);
vL_x = zeros(N,1);
vR_x = zeros(N,1);

vL_total = zeros(N,1);
vR_total = zeros(N,1);

v_meas = zeros(N,1);
w_meas = zeros(N,1);

stop_mode_log = zeros(N,1);

%% =========================
%  PID states
% =========================
intY = 0.0;
prevErrY = 0.0;
initY = false;

intW = 0.0;
prevErrW = 0.0;
initW = false;

%% =========================
%  For NLARX simulation
% =========================
% We simulate one step at a time using all input history up to k.
% This is slower, but easiest and safest for matching the nonlinear model.

uY_hist = zeros(N,1);
uX_hist = zeros(N,1);

%% =========================
%  Main closed-loop simulation
% =========================
for k = 1:N
    v_ref = v_ref_vec(k);
    w_ref = w_ref_vec(k);

    stop_mode = (abs(v_ref) <= SPEED_ENABLE_THRESHOLD) && ...
                (abs(w_ref) < TURN_ENABLE_THRESHOLD);
    stop_mode_log(k) = stop_mode;

    % ---------------------------------
    % 1) Feedforward LUT
    % ---------------------------------
    y_ff_raw = lookupYVoltage_matlab(v_ref);
    x_ff_raw = lookupXVoltage_matlab(w_ref);

    y_ff(k) = Y_NEUTRAL + Y_FF_GAIN * (y_ff_raw - Y_NEUTRAL);
    x_ff(k) = X_NEUTRAL + X_FF_GAIN * (x_ff_raw - X_NEUTRAL);

    % ---------------------------------
    % 2) PID correction
    % IMU 없음 -> w_meas_for_control = w_meas_encoder
    % ---------------------------------
    if stop_mode
        y_corr_v(k) = 0.0;
        x_corr_v(k) = 0.0;

        % reset like resetControllerAndHold()
        intY = 0.0; prevErrY = 0.0; initY = false;
        intW = 0.0; prevErrW = 0.0; initW = false;

        y_cmd(k) = Y_NEUTRAL + trim_y;
        x_cmd(k) = X_NEUTRAL + trim_x;
    else
        % ----- pidY -----
        if abs(v_ref) > SPEED_ENABLE_THRESHOLD
            [y_corr_v(k), intY, prevErrY, initY] = pid_update_matlab( ...
                v_ref, get_prev(v_meas, k), Ts, ...
                KpY, KiY, KdY, ...
                Y_CORR_MIN, Y_CORR_MAX, ...
                intY, prevErrY, initY);
        else
            y_corr_v(k) = 0.0;
            intY = 0.0; prevErrY = 0.0; initY = false;
        end

        % ----- pidW -----
        [x_corr_v(k), intW, prevErrW, initW] = pid_update_matlab( ...
            w_ref, get_prev(w_meas, k), Ts, ...
            KpW, KiW, KdW, ...
            X_CORR_MIN, X_CORR_MAX, ...
            intW, prevErrW, initW);

        % main.cpp has: x_corr_v = -pidW.update(...)
        x_corr_v(k) = -x_corr_v(k);

        % final command
        y_cmd(k) = y_ff(k) + y_corr_v(k) + trim_y;
        x_cmd(k) = x_ff(k) + x_corr_v(k) + trim_x;
    end

    % clamp to voltage range
    y_cmd(k) = clampf(y_cmd(k), Y_MIN_V, Y_MAX_V);
    x_cmd(k) = clampf(x_cmd(k), X_MIN_V, X_MAX_V);

    % ---------------------------------
    % 3) Plant simulation
    %    Use 4 SISO models approximately
    % ---------------------------------
    uY_hist(k) = y_cmd(k);
    uX_hist(k) = x_cmd(k);

    dataYk = iddata([], uY_hist(1:k), Ts);
    dataXk = iddata([], uX_hist(1:k), Ts);

    out_vL_y = sim(model_y_vL, dataYk);
    out_vR_y = sim(model_y_vR, dataYk);

    out_vL_x = sim(model_x_vL, dataXk);
    out_vR_x = sim(model_x_vR, dataXk);

    vL_y(k) = out_vL_y.OutputData(end);
    vR_y(k) = out_vR_y.OutputData(end);

    vL_x(k) = out_vL_x.OutputData(end);
    vR_x(k) = out_vR_x.OutputData(end);

    vL_total(k) = vL_y(k) + vL_x(k);
    vR_total(k) = vR_y(k) + vR_x(k);

    v_meas(k) = (vL_total(k) + vR_total(k)) / 2.0;
    w_meas(k) = (vR_total(k) - vL_total(k)) / WHEEL_SEPARATION;
end

%% =========================
%  Plots
% =========================
figure;
plot(t, v_ref_vec, 'LineWidth', 1.5); hold on;
plot(t, w_ref_vec, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Reference');
legend('v_{ref}', 'w_{ref}');
title('cmd\_vel reference');

figure;
plot(t, y_cmd, 'LineWidth', 1.5); hold on;
plot(t, x_cmd, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Voltage [V]');
legend('y_{cmd}', 'x_{cmd}');
title('Controller output voltages');

figure;
plot(t, v_ref_vec, '--', 'LineWidth', 1.5); hold on;
plot(t, v_meas, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Linear velocity');
legend('v_{ref}', 'v_{meas}');
title('Linear velocity tracking');

figure;
plot(t, w_ref_vec, '--', 'LineWidth', 1.5); hold on;
plot(t, w_meas, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Angular velocity');
legend('w_{ref}', 'w_{meas}');
title('Angular velocity tracking (encoder-only)');

figure;
plot(t, vL_total, 'LineWidth', 1.5); hold on;
plot(t, vR_total, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Wheel velocity');
legend('vL', 'vR');
title('Wheel velocities');

figure;
subplot(3,1,1);
plot(t, y_ff, 'LineWidth', 1.2); hold on;
plot(t, y_corr_v, 'LineWidth', 1.2);
grid on;
legend('y_{ff}', 'y_{corr}');
title('Y channel');

subplot(3,1,2);
plot(t, x_ff, 'LineWidth', 1.2); hold on;
plot(t, x_corr_v, 'LineWidth', 1.2);
grid on;
legend('x_{ff}', 'x_{corr}');
title('X channel');

subplot(3,1,3);
plot(t, stop_mode_log, 'LineWidth', 1.2);
grid on;
ylim([-0.1 1.1]);
title('Stop mode');

%% =========================
%  Helper functions
% =========================
function val = get_prev(x, k)
    if k <= 1
        val = 0.0;
    else
        val = x(k-1);
    end
end

function y = clampf(x, lo, hi)
    y = min(max(x, lo), hi);
end

function [out, integral, prevErr, initialized] = pid_update_matlab( ...
    target, measured, dt, kp, ki, kd, out_min, out_max, integral, prevErr, initialized)

    if dt <= 0
        out = 0.0;
        return;
    end

    err = target - measured;

    if ~initialized
        prevErr = err;
        initialized = true;
    end

    derr = (err - prevErr) / dt;
    integral_candidate = integral + err * dt;

    output_unsat = kp * err + ki * integral_candidate + kd * derr;
    out = output_unsat;

    if out > out_max
        out = out_max;
    elseif out < out_min
        out = out_min;
    else
        integral = integral_candidate;
    end

    prevErr = err;
end

function yv = lookupYVoltage_matlab(v_ref)
    bp = [-1.0000 -0.9000 -0.8000 -0.7000 -0.6000 -0.5000 -0.4000 -0.3000 ...
          -0.2000 -0.1500 -0.1000 -0.0800 -0.0600 -0.0400 -0.0200 0.0000 ...
           0.0200  0.0400  0.0600  0.0800  0.1000  0.1500  0.2000  0.3000 ...
           0.4000  0.5000  0.6000  0.7000  0.8000  0.9000  1.0000];

    table = [1.7890 1.8680 1.9470 2.0270 2.1060 2.1850 2.2640 2.3430 ...
             2.4230 2.4620 2.5020 2.5180 2.5330 2.5490 2.5650 2.6890 ...
             2.8130 2.8290 2.8450 2.8600 2.8760 2.9160 2.9550 3.0350 ...
             3.1140 3.1930 3.2720 3.3510 3.4310 3.5100 3.5890];

    yv = interp1_clip(bp, table, v_ref);
end

function xv = lookupXVoltage_matlab(w_ref)
    bp = [-1.0000 -0.9000 -0.8000 -0.7000 -0.6000 -0.5000 -0.4000 -0.3000 ...
          -0.2000 -0.1500 -0.1000 -0.0800 -0.0600 -0.0400 -0.0200 0.0000 ...
           0.0200  0.0400  0.0600  0.0800  0.1000  0.1500  0.2000  0.3000 ...
           0.4000  0.5000  0.6000  0.7000  0.8000  0.9000  1.0000];

    table = [1.9950 2.0570 2.1180 2.1800 2.2420 2.3030 2.3650 2.4260 ...
             2.4880 2.5190 2.5490 2.5620 2.5740 2.5860 2.5990 2.6950 ...
             2.7910 2.8040 2.8160 2.8280 2.8410 2.8720 2.9020 2.9640 ...
             3.0260 3.0870 3.1490 3.2100 3.2720 3.3340 3.3950];

    xv = interp1_clip(bp, table, w_ref);
end

function y = interp1_clip(bp, table, x)
    if x <= bp(1)
        y = table(1);
        return;
    end
    if x >= bp(end)
        y = table(end);
        return;
    end
    y = interp1(bp, table, x, 'linear');
end