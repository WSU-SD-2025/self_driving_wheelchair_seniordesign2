clear; clc; close all;

% Load CSV
T = readtable("/home/sejunmoon/self_driving_wheelchair/log/cmd_vel_angular_velocity.csv");

% Constants
X_NEUTRAL = 2.69;
X_MIN = 2.125;
X_MAX = 3.255;

% Use the larger side deviation for one global normalization
U_SCALE = max(X_MAX - X_NEUTRAL, X_NEUTRAL - X_MIN);

% Time
t = (T.time_ms - T.time_ms(1)) / 1000.0;

% Input preprocessing
u_center = T.x_voltage - X_NEUTRAL;
u = u_center / U_SCALE; % Roughly in [-1, 1]

% Outputs
vL = T.vL;
vR = T.vR;

% Remove invalid rows
valid = ~(isnan(t) | isnan(u) | isnan(vL) | isnan(vR));
t = t(valid);
u = u(valid);
vL = vL(valid);
vR = vR(valid);

% Estimate Ts
Ts = mean(diff(t));

%% Plot check
figure;
subplot(3,1,1); plot(t,u,'LineWidth',1.2); grid on; ylabel('X_voltage norm');
subplot(3,1,2); plot(t,vL,'LineWidth',1.2); grid on; ylabel('vL');
subplot(3,1,3); plot(t,vR,'LineWidth',1.2); grid on; ylabel('vR'); xlabel('Time (s)');

% Save processed CSV
P = table(t, u, vL, vR);
writetable(P, "/home/sejunmoon/self_driving_wheelchair/matlab/log/x_only_processed_normalized.csv");

% Separate iddata objects
data_vL = iddata(vL, u, Ts);
data_vR = iddata(vR, u, Ts);

save("x_only_iddata.mat", "data_vL", "data_vR");