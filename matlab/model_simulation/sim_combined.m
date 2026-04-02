clear; clc; close all;

%% ===== Load models =====
model_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/models';

load(fullfile(model_dir, 'model_y_vL.mat'));
load(fullfile(model_dir, 'model_y_vR.mat'));
load(fullfile(model_dir, 'model_x_vL.mat'));
load(fullfile(model_dir, 'model_x_vR.mat'));

%% ===== Basic settings =====
Ts = model_y_vL.Ts;            % 모든 모델 Ts가 같다고 가정
Tend = 15;                     % warm-up 포함 전체 시간 [s]
t = (0:Ts:Tend)';

Y_NEUTRAL = 2.689;
X_NEUTRAL = 2.6951;
WHEEL_SEPARATION = 0.575;

%% ===== Check sample times =====
if model_y_vR.Ts ~= Ts || model_x_vL.Ts ~= Ts || model_x_vR.Ts ~= Ts
    error('All model sample times must match.');
end

%% ===== Input scenario with warm-up =====
uY = Y_NEUTRAL * ones(size(t));
uX = X_NEUTRAL * ones(size(t));

% 0 ~ 3 s : neutral warm-up
%
% 3 ~ 6 s : forward only
% 6 ~ 9 s : forward + turn one direction
% 9 ~ 12 s: forward + opposite turn
% 12 ~ 13.5 s : neutral
% 13.5 ~ 15 s : backward + slight turn

uY(t >= 3    & t < 6)    = 3.05;
uY(t >= 6    & t < 9)    = 3.15;
uY(t >= 9    & t < 12)   = 3.15;
uY(t >= 12   & t < 13.5) = Y_NEUTRAL;
uY(t >= 13.5)            = 2.35;

uX(t >= 6    & t < 9)    = 2.45;
uX(t >= 9    & t < 12)   = 2.95;
uX(t >= 12   & t < 13.5) = X_NEUTRAL;
uX(t >= 13.5)            = 2.80;

%% ===== Simulate y and x separately =====
dataY = iddata([], uY, Ts);
dataX = iddata([], uX, Ts);

out_vL_y = sim(model_y_vL, dataY);
out_vR_y = sim(model_y_vR, dataY);

out_vL_x = sim(model_x_vL, dataX);
out_vR_x = sim(model_x_vR, dataX);

vL_y = out_vL_y.OutputData;
vR_y = out_vR_y.OutputData;
vL_x = out_vL_x.OutputData;
vR_x = out_vR_x.OutputData;

%% ===== Approximate combined response =====
vL_total = vL_y + vL_x;
vR_total = vR_y + vR_x;

v_body = (vL_total + vR_total) / 2;
w_body = (vR_total - vL_total) / WHEEL_SEPARATION;

%% ===== Plot =====
figure;
plot(t, uY, 'LineWidth', 1.5); hold on;
plot(t, uX, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Voltage [V]');
legend('uY', 'uX');
title('Combined simulation: input voltages (with warm-up)');
xlim([0 Tend]);

figure;
plot(t, vL_total, 'LineWidth', 1.5); hold on;
plot(t, vR_total, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Wheel velocity');
legend('vL total', 'vR total');
title('Combined simulation: wheel velocities');
xlim([0 Tend]);

figure;
plot(t, v_body, 'LineWidth', 1.5); hold on;
plot(t, w_body, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body response');
legend('v body', 'w body');
title('Combined simulation: body linear/angular response');
xlim([0 Tend]);

%% ===== zoom after warm-up only =====
figure;
plot(t, v_body, 'LineWidth', 1.5); hold on;
plot(t, w_body, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Body response');
legend('v body', 'w body');
title('Combined simulation: body response (after warm-up)');
xlim([3 Tend]);