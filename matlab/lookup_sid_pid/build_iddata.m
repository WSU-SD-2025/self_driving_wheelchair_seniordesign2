clear; clc; close all;

%% =========================
% User settings
% =========================
csv_file = '/home/sejunmoon/self_driving_wheelchair/log/wheelchair_log_linear_ground.csv';

% 'linear' or 'angular'
mode = 'linear';

% Input type:
% 'voltage'  -> linear: normalized y_voltage, angular: normalized x_voltage
% 'ref'      -> linear: ref_v, angular: ref_w
input_type = 'voltage';

train_ratio = 0.7;

% Encoder-wheel speed to wheelchair-wheel speed correction
speed_scale_div = 6.218;

% Optional smoothing
use_smoothing = false;
smooth_window = 5;

% Save iddata file
save_iddata_file = true;
save_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/systemid_data';

if ~isfolder(save_dir)
    mkdir(save_dir);
end

%% =========================
% Voltage normalization settings
% =========================
Y_NEUTRAL = 2.689;
X_NEUTRAL = 2.6951;

Y_MIN = 1.0;
Y_MAX = 4.2;
X_MIN = 1.0;
X_MAX = 4.2;

Y_SPAN = max(Y_MAX - Y_NEUTRAL, Y_NEUTRAL - Y_MIN);
X_SPAN = max(X_MAX - X_NEUTRAL, X_NEUTRAL - X_MIN);

%% =========================
% Load CSV
% =========================
T = readtable(csv_file);

required_cols = {'time_ms','ref_v','ref_w','y_voltage','x_voltage','vL','vR','v','w'};
for i = 1:numel(required_cols)
    if ~ismember(required_cols{i}, T.Properties.VariableNames)
        error('Missing required column: %s', required_cols{i});
    end
end

%% =========================
% Time
% =========================
time_s = (T.time_ms - T.time_ms(1)) / 1000.0;
dt = diff(time_s);

if any(dt <= 0)
    error('Time is not strictly increasing.');
end

Ts = mean(dt);

fprintf('===== TIME INFO =====\n');
fprintf('Samples   : %d\n', height(T));
fprintf('Ts mean   : %.6f s\n', Ts);
fprintf('Duration  : %.3f s\n', time_s(end));

%% =========================
% Raw signals
% =========================
ref_v     = T.ref_v;
ref_w     = T.ref_w;
y_voltage = T.y_voltage;
x_voltage = T.x_voltage;

% IMPORTANT: correct encoder-based speeds
v_body = T.v  / speed_scale_div;
w_body = T.w  / speed_scale_div;
vL     = T.vL / speed_scale_div;
vR     = T.vR / speed_scale_div;

if use_smoothing
    v_body = movmean(v_body, smooth_window);
    w_body = movmean(w_body, smooth_window);
    vL     = movmean(vL, smooth_window);
    vR     = movmean(vR, smooth_window);
end

%% =========================
% Normalized voltages for system ID
% =========================
y_voltage_norm = (y_voltage - Y_NEUTRAL) / Y_SPAN;
x_voltage_norm = (x_voltage - X_NEUTRAL) / X_SPAN;

% Clamp for safety
y_voltage_norm = max(min(y_voltage_norm, 1.0), -1.0);
x_voltage_norm = max(min(x_voltage_norm, 1.0), -1.0);

%% =========================
% Select input / output
% =========================
switch lower(mode)
    case 'linear'

        % Input
        switch lower(input_type)
            case 'voltage'
                u = y_voltage_norm;
                input_name = 'y_voltage_normalized';
            case 'ref'
                u = ref_v;
                input_name = 'ref_v';
            otherwise
                error('input_type must be ''voltage'' or ''ref''');
        end

        % Output: RIGHT wheel model
        y = vR;
        output_name = 'vR_scaled';

    case 'angular'

        % Input
        switch lower(input_type)
            case 'voltage'
                u = x_voltage_norm;
                input_name = 'x_voltage_normalized';
            case 'ref'
                u = ref_w;
                input_name = 'ref_w';
            otherwise
                error('input_type must be ''voltage'' or ''ref''');
        end

        % Default angular output
        y = w_body;
        output_name = 'w_body_scaled';

    otherwise
        error('mode must be ''linear'' or ''angular''');
end

%% =========================
% Build iddata
% =========================
z = iddata(y, u, Ts);

N = length(y);
Ntr = round(train_ratio * N);

z_train = z(1:Ntr);
z_val   = z(Ntr+1:end);

fprintf('\n===== IDDATA INFO =====\n');
fprintf('Mode           : %s\n', mode);
fprintf('Input          : %s\n', input_name);
fprintf('Output         : %s\n', output_name);
fprintf('Train samples  : %d\n', Ntr);
fprintf('Valid samples  : %d\n', N - Ntr);

%% =========================
% Simple plots for sanity check
% =========================
figure('Color','w');

subplot(4,1,1);
plot(time_s, u, 'LineWidth', 1.2);
grid on;
ylabel(input_name, 'Interpreter', 'none');
title(sprintf('%s input', mode), 'Interpreter', 'none');

subplot(4,1,2);
plot(time_s, y, 'LineWidth', 1.2);
grid on;
ylabel(output_name, 'Interpreter', 'none');
title(sprintf('%s output (scaled by %.3f)', output_name, speed_scale_div), 'Interpreter', 'none');

subplot(4,1,3);
if strcmpi(mode, 'linear')
    plot(time_s, y_voltage, 'LineWidth', 1.2); hold on;
    plot(time_s, y_voltage_norm, 'LineWidth', 1.2);
    ylabel('y voltage');
    legend('y voltage','y voltage norm','Location','best');
    title('Linear voltage vs normalized input');
else
    plot(time_s, x_voltage, 'LineWidth', 1.2); hold on;
    plot(time_s, x_voltage_norm, 'LineWidth', 1.2);
    ylabel('x voltage');
    legend('x voltage','x voltage norm','Location','best');
    title('Angular voltage vs normalized input');
end
grid on;

subplot(4,1,4);
if strcmpi(mode, 'linear')
    plot(time_s, vL, 'LineWidth', 1.2); hold on;
    plot(time_s, vR, 'LineWidth', 1.2);
    plot(time_s, v_body, '--', 'LineWidth', 1.0);
    ylabel('wheel/body speeds');
    legend('vL','vR','v\_body','Location','best');
else
    plot(time_s, v_body, 'LineWidth', 1.2); hold on;
    plot(time_s, w_body, 'LineWidth', 1.2);
    ylabel('body states');
    legend('v','w','Location','best');
end
grid on;
xlabel('Time (s)');

%% =========================
% Save iddata for GUI use
% =========================
if save_iddata_file
    save_name = 'iddata_linear_voltage_vR.mat';
    save_path = fullfile(save_dir, save_name);

    save(save_path, ...
        'z', 'z_train', 'z_val', ...
        'Ts', 'time_s', ...
        'u', 'y', ...
        'v_body', 'w_body', 'vL', 'vR', ...
        'ref_v', 'ref_w', ...
        'y_voltage', 'x_voltage', ...
        'y_voltage_norm', 'x_voltage_norm', ...
        'csv_file', 'mode', 'input_type', 'input_name', 'output_name', ...
        'speed_scale_div', ...
        'Y_NEUTRAL', 'X_NEUTRAL', ...
        'Y_MIN', 'Y_MAX', 'X_MIN', 'X_MAX', ...
        'Y_SPAN', 'X_SPAN');

    fprintf('\nSaved iddata to:\n%s\n', save_path);
end

%% =========================
% Helpful notes
% =========================
disp(' ');
disp('Workspace variables ready for System Identification GUI:');
disp('  z        -> full iddata');
disp('  z_train  -> training iddata');
disp('  z_val    -> validation iddata');
disp('  u        -> selected input');
disp('  y        -> selected output');