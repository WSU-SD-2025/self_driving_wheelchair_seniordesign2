clear; clc; close all;

%% =========================
% User settings
% =========================
csv_file = '/home/sejunmoon/self_driving_wheelchair/log/wheelchair_log_linear_ground.csv';

% 'linear' or 'angular'
mode = 'linear';

% Use the last portion of each step segment as steady-state
steady_ratio = 0.4;   % use last 40% of each segment
min_seg_samples = 10;

% Output file names
linear_out_file  = 'lookup_table_linear.mat';
angular_out_file = 'lookup_table_angular.mat';

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

time_s    = (T.time_ms - T.time_ms(1)) / 1000.0;
ref_v     = T.ref_v;
ref_w     = T.ref_w;
y_voltage = T.y_voltage;
x_voltage = T.x_voltage;

speed_scale_div = 6.218;

v_body    = T.v / speed_scale_div;
w_body    = T.w / speed_scale_div;
vL = T.vL / speed_scale_div;
vR = T.vR / speed_scale_div;

%% =========================
% Choose reference signal by mode
% =========================
switch lower(mode)
    case 'linear'
        ref_sig = ref_v;
        input_name = 'ref_v';
        output_name = 'v';
    case 'angular'
        ref_sig = ref_w;
        input_name = 'ref_w';
        output_name = 'w';
    otherwise
        error('mode must be ''linear'' or ''angular''');
end

%% =========================
% Find step segments
% =========================
change_idx = [1; find(abs(diff(ref_sig)) > 1e-9) + 1; height(T) + 1];

segments = struct( ...
    'idx_start', {}, ...
    'idx_end', {}, ...
    'ref_cmd', {}, ...
    'mean_y_voltage', {}, ...
    'mean_x_voltage', {}, ...
    'mean_v', {}, ...
    'mean_w', {}, ...
    'mean_vL', {}, ...
    'mean_vR', {}, ...
    'duration_sec', {} ...
);

seg_count = 0;

for k = 1:length(change_idx)-1
    i1 = change_idx(k);
    i2 = change_idx(k+1) - 1;

    if i2 <= i1
        continue;
    end

    seg_len = i2 - i1 + 1;
    if seg_len < min_seg_samples
        continue;
    end

    ss_len = max(min_seg_samples, round(seg_len * steady_ratio));
    ss_start = i2 - ss_len + 1;
    ss_idx = ss_start:i2;

    seg_count = seg_count + 1;
    segments(seg_count).idx_start = i1;
    segments(seg_count).idx_end = i2;
    segments(seg_count).ref_cmd = ref_sig(i1);
    segments(seg_count).mean_y_voltage = mean(y_voltage(ss_idx));
    segments(seg_count).mean_x_voltage = mean(x_voltage(ss_idx));
    segments(seg_count).mean_v = mean(v_body(ss_idx));
    segments(seg_count).mean_w = mean(w_body(ss_idx));
    segments(seg_count).mean_vL = mean(vL(ss_idx));
    segments(seg_count).mean_vR = mean(vR(ss_idx));
    segments(seg_count).duration_sec = time_s(i2) - time_s(i1);
end

if isempty(segments)
    error('No valid segments found.');
end

%% =========================
% Convert to table
% =========================
seg_table = struct2table(segments);

disp('===== Extracted step segments =====');
disp(seg_table);

%% =========================
% Build lookup table
% =========================
switch lower(mode)
    case 'linear'
        lookup_table = table( ...
            seg_table.ref_cmd, ...
            seg_table.mean_y_voltage, ...
            seg_table.mean_v, ...
            seg_table.mean_w, ...
            seg_table.mean_vL, ...
            seg_table.mean_vR, ...
            'VariableNames', {'ref_v','y_voltage','v','w','vL','vR'});

        % sort by ref_v
        lookup_table = sortrows(lookup_table, 'ref_v');

        % save
        save(linear_out_file, 'lookup_table', 'seg_table', 'csv_file');
        fprintf('\nSaved linear lookup table to %s\n', linear_out_file);

    case 'angular'
        lookup_table = table( ...
            seg_table.ref_cmd, ...
            seg_table.mean_x_voltage, ...
            seg_table.mean_w, ...
            seg_table.mean_v, ...
            seg_table.mean_vL, ...
            seg_table.mean_vR, ...
            'VariableNames', {'ref_w','x_voltage','w','v','vL','vR'});

        % sort by ref_w
        lookup_table = sortrows(lookup_table, 'ref_w');

        % save
        save(angular_out_file, 'lookup_table', 'seg_table', 'csv_file');
        fprintf('\nSaved angular lookup table to %s\n', angular_out_file);
end

%% =========================
% Plot raw signal + segment means
% =========================
figure('Color','w');

subplot(3,1,1);
plot(time_s, ref_sig, 'LineWidth', 1.2);
grid on;
ylabel(input_name);
title(sprintf('%s step segmentation', mode));

subplot(3,1,2);
if strcmpi(mode, 'linear')
    plot(time_s, y_voltage, 'LineWidth', 1.2); hold on;
    yline(mean(y_voltage), '--');
    ylabel('y voltage');
else
    plot(time_s, x_voltage, 'LineWidth', 1.2); hold on;
    yline(mean(x_voltage), '--');
    ylabel('x voltage');
end
grid on;

subplot(3,1,3);
if strcmpi(mode, 'linear')
    plot(time_s, v_body, 'LineWidth', 1.2);
    ylabel('v body');
else
    plot(time_s, w_body, 'LineWidth', 1.2);
    ylabel('w body');
end
grid on;
xlabel('Time (s)');

%% =========================
% Plot lookup table
% =========================
figure('Color','w');

if strcmpi(mode, 'linear')
    subplot(2,1,1);
    plot(lookup_table.ref_v, lookup_table.y_voltage, '-o', 'LineWidth', 1.5);
    grid on;
    xlabel('ref\_v');
    ylabel('y voltage');
    title('Linear lookup: ref\_v -> y\_voltage');

    subplot(2,1,2);
    plot(lookup_table.ref_v, lookup_table.v, '-o', 'LineWidth', 1.5);
    grid on;
    xlabel('ref\_v');
    ylabel('measured v');
    title('Linear steady-state tracking');
else
    subplot(2,1,1);
    plot(lookup_table.ref_w, lookup_table.x_voltage, '-o', 'LineWidth', 1.5);
    grid on;
    xlabel('ref\_w');
    ylabel('x voltage');
    title('Angular lookup: ref\_w -> x\_voltage');

    subplot(2,1,2);
    plot(lookup_table.ref_w, lookup_table.w, '-o', 'LineWidth', 1.5);
    grid on;
    xlabel('ref\_w');
    ylabel('measured w');
    title('Angular steady-state tracking');
end