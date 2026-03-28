clear; clc; close all;

%% Load NLARX models
S1 = load('/home/sejunmoon/self_driving_wheelchair/matlab/model_y_vL.mat');
S2 = load('/home/sejunmoon/self_driving_wheelchair/matlab/model_y_vR.mat');

model_y_vL = S1.model_y_vL;
model_y_vR = S2.model_y_vR;

%% Sample time
Ts = model_y_vL.Ts;
if Ts <= 0
    error('Model sample time Ts is invalid');
end

%% Simulation settings
Tend = 12;
N = round(Tend / Ts) + 1;
t = (0:N-1)' * Ts;

%% Target speed
v_target = zeros(N,1);
v_target(t >= 1.0) = 0.15;

%% PID gains
Kp = 0.8;
Ki = 0.00;
Kd = 0.00;

%% Normalized control limits
u_min = -1.0;
u_max = 1.0;

%% Deadzone compensation
use_deadzone = false;
deadzone_u = 0.05;

%% Storage
u_cmd    = zeros(N,1);
vL_sim   = zeros(N,1);
vR_sim   = zeros(N,1);
v_actual = zeros(N,1);
e        = zeros(N,1);

%% PID states
integral_e = 0.0;
prev_e = 0.0;

%% Closed-loop simulation
for k = 2:N

    % Current measured/simulated forward speed
    v_actual(k-1) = 0.5 * (vL_sim(k-1) + vR_sim(k-1));

    % Error
    e(k) = v_target(k) - v_actual(k-1);

    % PID terms
    integral_candidate = integral_e + e(k) * Ts;
    derivative_e = (e(k) - prev_e) / Ts;

    u_unsat = Kp * e(k) + Ki * integral_candidate + Kd * derivative_e;

    % Saturation
    u_sat = min(max(u_unsat, u_min), u_max);

    % Anti-windup
    if abs(u_unsat - u_sat) < 1e-9
        integral_e = integral_candidate;
    end

    % Optional deadzone compensation
    if use_deadzone && abs(u_sat) > 0 && abs(u_sat) < deadzone_u
        u_sat = sign(u_sat) * deadzone_u;
    end

    u_cmd(k) = u_sat;
    prev_e = e(k);

    % Build input history as iddata
    z_in = iddata([], u_cmd(1:k), Ts);

    % Simulate NLARX models
    zL = sim(model_y_vL, z_in);
    zR = sim(model_y_vR, z_in);

    % Take latest outputs
    vL_sim(k) = zL.OutputData(end);
    vR_sim(k) = zR.OutputData(end);
end

% Final actual speed update
v_actual(end) = 0.5 * (vL_sim(end) + vR_sim(end));

%% Plot results
figure;
subplot(4,1,1);
plot(t, v_target, 'LineWidth', 1.5); hold on;
plot(t, v_actual, 'LineWidth', 1.5);
grid on;
ylabel('v (m/s)');
legend('v target','v actual','Location','best');
title('Y-axis Closed-Loop PID with NLARX Plant');

subplot(4,1,2);
plot(t, u_cmd, 'LineWidth', 1.5);
grid on;
ylabel('u_y norm');
ylim([-1.1 1.1]);

subplot(4,1,3);
plot(t, vL_sim, 'LineWidth', 1.5); hold on;
plot(t, vR_sim, 'LineWidth', 1.5);
grid on;
ylabel('wheel v');
legend('vL','vR','Location','best');

subplot(4,1,4);
plot(t, v_target - v_actual, 'LineWidth', 1.5);
grid on;
ylabel('error');
xlabel('Time (s)');

%% Basic performance printout
steady_idx = t >= 8.0;
ss_error = mean(v_target(steady_idx) - v_actual(steady_idx));
fprintf('Ts = %.5f s\n', Ts);
fprintf('Mean steady-state error after 8s = %.5f m/s\n', ss_error);
fprintf('Peak actual speed = %.5f m/s\n', max(v_actual));