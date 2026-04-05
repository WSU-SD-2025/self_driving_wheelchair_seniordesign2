clc; clear; close all;

%% =========================
%  USER SETTINGS
% ==========================
% 모델 파일 이름은 네가 가진 이름으로 바꿔
yl = load("/home/sejunmoon/self_driving_wheelchair/matlab/models/y_model_L.mat");   % 예: model_y_L
yr = load("/home/sejunmoon/self_driving_wheelchair/matlab/models/y_model_R.mat");   % 예: model_y_R
xl = load("/home/sejunmoon/self_driving_wheelchair/matlab/models/x_model_L.mat");   % 예: model_x_L
xr = load("/home/sejunmoon/self_driving_wheelchair/matlab/models/x_model_R.mat");   % 예: model_x_R

% ---- 여기서 실제 변수명 맞춰줘 ----
% 예를 들어 mat 파일 안에 model 변수가 이 이름이라면 그대로
mdl_yL = yl.model_y_L;
mdl_yR = yr.model_y_R;
mdl_xL = xl.model_x_L;
mdl_xR = xr.model_x_R;

%% =========================
%  CONSTANTS
% ==========================
Ts = mdl_yL.Ts;              % 샘플타임
Tend = 30;                   % 총 시뮬 시간
t = (0:Ts:Tend)';
N = numel(t);

WHEEL_SEPARATION = 0.575;

Y_NEUTRAL = 2.689;
X_NEUTRAL = 2.705;

Y_MIN = 1.0; Y_MAX = 4.2;
X_MIN = 1.0; X_MAX = 4.2;

% PID 출력 제한: "전압 보정량"
Y_CORR_MIN = -0.20;  Y_CORR_MAX = 0.20;
X_CORR_MIN = -0.18;  X_CORR_MAX = 0.18;

%% =========================
%  PID GAINS (초기값)
% ==========================
% Linear
Kp_y = 0.35;
Ki_y = 0.08;
Kd_y = 0.00;

% Angular
Kp_x = 0.18;
Ki_x = 0.02;
Kd_x = 0.00;

%% =========================
%  REFERENCE (cmd_vel 형태)
% ==========================
v_ref = zeros(N,1);
w_ref = zeros(N,1);

% 예시 시나리오
for k = 1:N
    tk = t(k);

    if tk >= 2 && tk < 8
        v_ref(k) = 0.30;
        w_ref(k) = 0.00;

    elseif tk >= 10 && tk < 15
        v_ref(k) = 0.00;
        w_ref(k) = 0.35;

    elseif tk >= 17 && tk < 24
        v_ref(k) = 0.25;
        w_ref(k) = 0.20;

    else
        v_ref(k) = 0.00;
        w_ref(k) = 0.00;
    end
end

%% =========================
%  LOOKUP TABLE (네 코드 기준)
% ==========================
v_bp = [-1.0000 -0.9000 -0.8000 -0.7000 -0.6000 -0.5000 -0.4000 -0.3000 ...
        -0.2000 -0.1500 -0.1000 -0.0800 -0.0600 -0.0400 -0.0200 ...
         0.0000 ...
         0.0200  0.0400  0.0600  0.0800  0.1000  0.1500  0.2000  0.3000 ...
         0.4000  0.5000  0.6000  0.7000  0.8000  0.9000  1.0000];

y_table = [1.7890 1.8680 1.9470 2.0270 2.1060 2.1850 2.2640 2.3430 ...
           2.4230 2.4620 2.5020 2.5180 2.5330 2.5490 2.5650 ...
           2.6890 ...
           2.8130 2.8290 2.8450 2.8600 2.8760 2.9160 2.9550 3.0350 ...
           3.1140 3.1930 3.2720 3.3510 3.4310 3.5100 3.5890];

w_bp = [-1.0000 -0.9000 -0.8000 -0.7000 -0.6000 -0.5000 -0.4000 -0.3000 ...
        -0.2000 -0.1500 -0.1000 -0.0800 -0.0600 -0.0400 -0.0200 ...
         0.0000 ...
         0.0200  0.0400  0.0600  0.0800  0.1000  0.1500  0.2000  0.3000 ...
         0.4000  0.5000  0.6000  0.7000  0.8000  0.9000  1.0000];

x_table = [2.0050 2.0670 2.1280 2.1900 2.2520 2.3130 2.3750 2.4360 ...
           2.4980 2.5290 2.5590 2.5720 2.5840 2.5960 2.6090 ...
           2.7050 ...
           2.8010 2.8140 2.8260 2.8380 2.8510 2.8820 2.9120 2.9740 ...
           3.0360 3.0970 3.1590 3.2200 3.2820 3.3440 3.4050];

%% =========================
%  PREALLOC
% ==========================
y_ff      = zeros(N,1);
x_ff      = zeros(N,1);
y_corr    = zeros(N,1);
x_corr    = zeros(N,1);
y_cmd     = zeros(N,1);
x_cmd     = zeros(N,1);

vL_y      = zeros(N,1);
vR_y      = zeros(N,1);
vL_x      = zeros(N,1);
vR_x      = zeros(N,1);
vL        = zeros(N,1);
vR        = zeros(N,1);

v_meas    = zeros(N,1);
w_meas    = zeros(N,1);

% PID states
int_y = 0;
prev_err_y = 0;
init_y = false;

int_x = 0;
prev_err_x = 0;
init_x = false;

%% =========================
%  NLARX model states
% ==========================
% compareOptions/predictOptions 쓸 수도 있지만,
% step-by-step로 하려면 predict를 1-step씩 반복하는 식보다
% idnlarx에 sim를 계속 단일 입력으로 넣는 게 불편할 수 있음.
% 그래서 여기서는 매 step마다 최근 입력 히스토리를 쌓아
% predict 기반으로 간단히 근사.

na = 10;  % 필요시 수정
nb = 10;  % 필요시 수정

u_y_hist = zeros(nb,1);
u_x_hist = zeros(nb,1);

yL_hist = zeros(na,1);
yR_hist = zeros(na,1);
xL_hist = zeros(na,1);
xR_hist = zeros(na,1);

%% =========================
%  MAIN LOOP
% ==========================
for k = 1:N

    % 1) feedforward from LUT
    y_ff(k) = interp1(v_bp, y_table, v_ref(k), 'linear', 'extrap');
    x_ff(k) = interp1(w_bp, x_table, w_ref(k), 'linear', 'extrap');

    % 2) PID
    if k == 1
        v_now = 0;
        w_now = 0;
    else
        v_now = v_meas(k-1);
        w_now = w_meas(k-1);
    end

    % ---- linear PID ----
    err_y = v_ref(k) - v_now;
    if ~init_y
        prev_err_y = err_y;
        init_y = true;
    end

    der_y = (err_y - prev_err_y) / Ts;
    int_candidate_y = int_y + err_y * Ts;
    out_unsat_y = Kp_y*err_y + Ki_y*int_candidate_y + Kd_y*der_y;
    out_y = min(max(out_unsat_y, Y_CORR_MIN), Y_CORR_MAX);

    if abs(out_y - out_unsat_y) < 1e-9
        int_y = int_candidate_y;
    end
    prev_err_y = err_y;
    y_corr(k) = out_y;

    % ---- angular PID ----
    err_x = w_ref(k) - w_now;
    if ~init_x
        prev_err_x = err_x;
        init_x = true;
    end

    der_x = (err_x - prev_err_x) / Ts;
    int_candidate_x = int_x + err_x * Ts;
    out_unsat_x = Kp_x*err_x + Ki_x*int_candidate_x + Kd_x*der_x;
    out_x = min(max(out_unsat_x, X_CORR_MIN), X_CORR_MAX);

    if abs(out_x - out_unsat_x) < 1e-9
        int_x = int_candidate_x;
    end
    prev_err_x = err_x;
    x_corr(k) = out_x;

    % 네 ESP32 코드에서는 x쪽에 -pidW.update(...)가 들어감
    % 그래서 부호를 실제 코드와 맞추려면 보통 여기서도 반영해줘야 함
    x_corr(k) = -x_corr(k);

    % 3) final command voltages
    y_cmd(k) = min(max(y_ff(k) + y_corr(k), Y_MIN), Y_MAX);
    x_cmd(k) = min(max(x_ff(k) + x_corr(k), X_MIN), X_MAX);

    % 4) delta voltage relative to neutral
    dy = y_cmd(k) - Y_NEUTRAL;
    dx = x_cmd(k) - X_NEUTRAL;

    % input history update
    u_y_hist = [dy; u_y_hist(1:end-1)];
    u_x_hist = [dx; u_x_hist(1:end-1)];

    % 5) wheel model prediction
    % 여기 부분은 네 모델 차수/구조에 따라 조금 수정될 수 있음
    % 가장 단순하게 recent input/output 기반 iddata를 만들어 predict 사용

    z_yL = iddata([], flipud(u_y_hist), Ts);
    z_yR = iddata([], flipud(u_y_hist), Ts);
    z_xL = iddata([], flipud(u_x_hist), Ts);
    z_xR = iddata([], flipud(u_x_hist), Ts);

    % 출력 히스토리 연결
    % predict가 히스토리를 자동으로 완전히 먹는 방식이 아니라서
    % 모델에 따라 이 부분은 수정이 필요할 수 있음.
    % 안 되면 sim(mdl, input) 방식으로 바꾸면 됨.

    try
        ytemp = sim(mdl_yL, z_yL);
        vL_y(k) = ytemp.OutputData(end);

        ytemp = sim(mdl_yR, z_yR);
        vR_y(k) = ytemp.OutputData(end);

        ytemp = sim(mdl_xL, z_xL);
        vL_x(k) = ytemp.OutputData(end);

        ytemp = sim(mdl_xR, z_xR);
        vR_x(k) = ytemp.OutputData(end);
    catch
        % 만약 sim가 이 형태로 안 먹으면 fallback
        if k == 1
            vL_y(k)=0; vR_y(k)=0; vL_x(k)=0; vR_x(k)=0;
        else
            vL_y(k)=vL_y(k-1); vR_y(k)=vR_y(k-1);
            vL_x(k)=vL_x(k-1); vR_x(k)=vR_x(k-1);
        end
    end

    % 6) combine wheel contributions
    vL(k) = vL_y(k) + vL_x(k);
    vR(k) = vR_y(k) + vR_x(k);

    % 7) body velocity
    v_meas(k) = (vL(k) + vR(k)) / 2;
    w_meas(k) = (vR(k) - vL(k)) / WHEEL_SEPARATION;
end

%% =========================
%  PLOTS
% ==========================
figure;
plot(t, v_ref, 'LineWidth', 1.8); hold on;
plot(t, v_meas, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Linear velocity (m/s)');
legend('v ref', 'v meas');
title('Linear Velocity Tracking');

figure;
plot(t, w_ref, 'LineWidth', 1.8); hold on;
plot(t, w_meas, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');
legend('w ref', 'w meas');
title('Angular Velocity Tracking');

figure;
plot(t, y_cmd, 'LineWidth', 1.8); hold on;
plot(t, x_cmd, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
legend('y cmd', 'x cmd');
title('Command Voltages');

figure;
plot(t, vL, 'LineWidth', 1.8); hold on;
plot(t, vR, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Wheel speed');
legend('vL', 'vR');
title('Wheel Speeds');