clear; clc; close all;

%% ===== Load models =====
model_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/models';

load(fullfile(model_dir, 'model_y_vL.mat'));
load(fullfile(model_dir, 'model_y_vR.mat'));

%% ===== Basic settings =====
Ts = model_y_vL.Ts;
Tend = 13;                     % warm-up 포함 전체 시간 [s]
t = (0:Ts:Tend)';

Y_NEUTRAL = 2.689;

%% ===== Input scenario: y voltage only with warm-up =====
uY = Y_NEUTRAL * ones(size(t));

% 0 ~ 3 s : neutral warm-up
% 3 ~ 5 s : mild forward
% 5 ~ 7 s : medium forward
% 7 ~ 9 s : stronger forward
% 9 ~ 10.5 s : back to neutral
% 10.5 s ~ end : backward

uY(t >= 3   & t < 5)    = 2.90;
uY(t >= 5   & t < 7)    = 3.10;
uY(t >= 7   & t < 9)    = 3.30;
uY(t >= 9   & t < 10.5) = Y_NEUTRAL;
uY(t >= 10.5)           = 2.35;

%% ===== Simulate =====
dataY = iddata([], uY, Ts);

out_vL = sim(model_y_vL, dataY);
out_vR = sim(model_y_vR, dataY);

vL = out_vL.OutputData;
vR = out_vR.OutputData;

v_body = (vL + vR) / 2;

%% ===== Plot =====
figure;
plot(t, uY, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Y Voltage [V]');
title('Input: Y voltage only (with warm-up)');

figure;
plot(t, vL, 'LineWidth', 1.5); hold on;
plot(t, vR, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Wheel velocity');
legend('vL', 'vR');
title('Y-only simulation: wheel velocities');
xlim([0 Tend]);

figure;
plot(t, v_body, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body linear velocity');
title('Y-only simulation: body linear velocity');
xlim([0 Tend]);

%% ===== zoom after warm-up only =====
figure;
plot(t, v_body, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body linear velocity');
title('Y-only simulation: body linear velocity (after warm-up)');
xlim([3 Tend]);