clear; clc; close all;

%% =========================
% User settings
% =========================
csv_file = '/home/sejunmoon/self_driving_wheelchair/log/wheelchair_log_angular_ground.csv';

% Save folder
lut_dir = '/home/sejunmoon/self_driving_wheelchair/matlab/LUT';
if ~isfolder(lut_dir)
    mkdir(lut_dir);
end

angular_out_mat = fullfile(lut_dir, 'lookup_table_angular.mat');
angular_out_csv = fullfile(lut_dir, 'lookup_table_angular.csv');

% Encoder-wheel to wheelchair-wheel speed correction
speed_scale_div = 6.218;

% Use the last portion of each step segment as steady-state
steady_ratio = 0.4;      % use last 40% of each segment
min_seg_samples = 10;

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
% Read signals
% =========================
time_s    = (T.time_ms - T.time_ms(1)) / 1000.0;
ref_v     = T.ref_v;
ref_w     = T.ref_w;
y_voltage = T.y_voltage;
x_voltage = T.x_voltage;

% Correct scaled speeds
v_body = T.v  / speed_scale_div;
w_body = T.w  / speed_scale_div;
vL     = T.vL / speed_scale_div;
vR     = T.vR / speed_scale_div;

%% =========================
% Find step segments using ref_w
% =========================
ref_sig = ref_w;
change_idx = [1; find(abs(diff(ref_sig)) > 1e-9) + 1; height(T) + 1];

segments = struct( ...
    'idx_start', {}, ...
    'idx_end', {}, ...
    'ref_w', {}, ...
    'mean_x_voltage', {}, ...
    'mean_y_voltage', {}, ...
    'mean_w', {}, ...
    'mean_v', {}, ...
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
    segments(seg_count).ref_w = ref_sig(i1);
    segments(seg_count).mean_x_voltage = mean(x_voltage(ss_idx));
    segments(seg_count).mean_y_voltage = mean(y_voltage(ss_idx));
    segments(seg_count).mean_w = mean(w_body(ss_idx));
    segments(seg_count).mean_v = mean(v_body(ss_idx));
    segments(seg_count).mean_vL = mean(vL(ss_idx));
    segments(seg_count).mean_vR = mean(vR(ss_idx));
    segments(seg_count).duration_sec = time_s(i2) - time_s(i1);
end

if isempty(segments)
    error('No valid angular segments found.');
end

%% =========================
% Convert to table
% =========================
seg_table = struct2table(segments);

disp('===== Extracted angular step segments =====');
disp(seg_table);

%% =========================
% Build angular lookup table
% =========================
lookup_table = table( ...
    seg_table.ref_w, ...
    seg_table.mean_x_voltage, ...
    seg_table.mean_w, ...
    seg_table.mean_v, ...
    seg_table.mean_vL, ...
    seg_table.mean_vR, ...
    'VariableNames', {'ref_w','x_voltage','w','v','vL','vR'});

% Sort by ref_w
lookup_table = sortrows(lookup_table, 'ref_w');

%% =========================
% Optional: merge duplicate ref_w rows by averaging
% (safe if repeated zero or repeated test points exist)
% =========================
[G, unique_ref_w] = findgroups(lookup_table.ref_w);

lookup_table_merged = table;
lookup_table_merged.ref_w      = unique_ref_w;
lookup_table_merged.x_voltage  = splitapply(@mean, lookup_table.x_voltage, G);
lookup_table_merged.w          = splitapply(@mean, lookup_table.w, G);
lookup_table_merged.v          = splitapply(@mean, lookup_table.v, G);
lookup_table_merged.vL         = splitapply(@mean, lookup_table.vL, G);
lookup_table_merged.vR         = splitapply(@mean, lookup_table.vR, G);

%% =========================
% Save results
% =========================
save(angular_out_mat, ...
    'lookup_table', ...
    'lookup_table_merged', ...
    'seg_table', ...
    'csv_file', ...
    'speed_scale_div');

writetable(lookup_table_merged, angular_out_csv);

fprintf('\nSaved angular lookup MAT to:\n%s\n', angular_out_mat);
fprintf('Saved angular lookup CSV to:\n%s\n', angular_out_csv);

%% =========================
% Plot raw signals
% =========================
figure('Color','w');

subplot(4,1,1);
plot(time_s, ref_w, 'LineWidth', 1.2);
grid on;
ylabel('ref\_w');
title(sprintf('Angular step segmentation (scaled by %.3f)', speed_scale_div));

subplot(4,1,2);
plot(time_s, x_voltage, 'LineWidth', 1.2); hold on;
yline(mean(x_voltage), '--');
grid on;
ylabel('x voltage');

subplot(4,1,3);
plot(time_s, w_body, 'LineWidth', 1.2);
grid on;
ylabel('scaled w');

subplot(4,1,4);
plot(time_s, v_body, 'LineWidth', 1.2);
grid on;
ylabel('scaled v');
xlabel('Time (s)');

%% =========================
% Plot lookup table
% =========================
figure('Color','w');

subplot(2,1,1);
plot(lookup_table_merged.ref_w, lookup_table_merged.x_voltage, '-o', 'LineWidth', 1.5);
grid on;
xlabel('ref\_w');
ylabel('x voltage');
title('Angular lookup: ref\_w -> x\_voltage');

subplot(2,1,2);
plot(lookup_table_merged.ref_w, lookup_table_merged.w, '-o', 'LineWidth', 1.5);
grid on;
xlabel('ref\_w');
ylabel('scaled measured w');
title('Angular steady-state tracking');

%% =========================
% Helpful printouts for C++ LUT update
% =========================
disp(' ');
disp('===== C++ copy-paste arrays (Angular LUT) =====');

fprintf('static const float w_bp[%d] = {\n', height(lookup_table_merged));
for i = 1:height(lookup_table_merged)
    if i < height(lookup_table_merged)
        fprintf('    %.4ff,\n', lookup_table_merged.ref_w(i));
    else
        fprintf('    %.4ff\n', lookup_table_merged.ref_w(i));
    end
end
fprintf('};\n\n');

fprintf('static const float x_table[%d] = {\n', height(lookup_table_merged));
for i = 1:height(lookup_table_merged)
    if i < height(lookup_table_merged)
        fprintf('    %.4ff,\n', lookup_table_merged.x_voltage(i));
    else
        fprintf('    %.4ff\n', lookup_table_merged.x_voltage(i));
    end
end
fprintf('};\n');