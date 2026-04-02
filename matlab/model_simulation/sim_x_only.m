clear; clc; close all;

%% ===== Load models =====
model_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/models';

load(fullfile(model_dir, 'model_x_vL.mat'));
load(fullfile(model_dir, 'model_x_vR.mat'));

%% ===== Basic settings =====
Ts = model_x_vL.Ts;
Tend = 13;                     % warm-up 포함 전체 시간 [s]
t = (0:Ts:Tend)';

X_NEUTRAL = 2.6951;
WHEEL_SEPARATION = 0.575;

%% ===== Input scenario: x voltage only with warm-up =====
uX = X_NEUTRAL * ones(size(t));

% 0 ~ 3 s : neutral warm-up
% 3 ~ 5 s : mild turn one direction
% 5 ~ 7 s : stronger turn same direction
% 7 ~ 9 s : opposite turn
% 9 ~ 10.5 s : stronger opposite turn
% 10.5 s ~ end : back to neutral

uX(t >= 3   & t < 5)    = 2.50;
uX(t >= 5   & t < 7)    = 2.35;
uX(t >= 7   & t < 9)    = 2.90;
uX(t >= 9   & t < 10.5) = 3.10;
uX(t >= 10.5)           = X_NEUTRAL;

%% ===== Simulate =====
dataX = iddata([], uX, Ts);

out_vL = sim(model_x_vL, dataX);
out_vR = sim(model_x_vR, dataX);

vL = out_vL.OutputData;
vR = out_vR.OutputData;

v_body = (vL + vR) / 2;
w_body = (vR - vL) / WHEEL_SEPARATION;

%% ===== Plot =====
figure;
plot(t, uX, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('X Voltage [V]');
title('Input: X voltage only (with warm-up)');

figure;
plot(t, vL, 'LineWidth', 1.5); hold on;
plot(t, vR, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Wheel velocity');
legend('vL', 'vR');
title('X-only simulation: wheel velocities');
xlim([0 Tend]);

figure;
plot(t, v_body, 'LineWidth', 1.5); hold on;
plot(t, w_body, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body response');
legend('v body', 'w body');
title('X-only simulation: body response');
xlim([0 Tend]);

%% ===== zoom after warm-up only =====
figure;
plot(t, w_body, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body angular velocity');
title('X-only simulation: body angular velocity (after warm-up)');
xlim([3 Tend]);