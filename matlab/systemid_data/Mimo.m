clc; clear; close all;

csv_file = "/home/sejunmoon/Desktop/cmd_vel_combined.csv";

Y_NEUTRAL = 2.689;
X_NEUTRAL = 2.690;   
SCALE_DIV = 6.218;

T = readtable(csv_file);

t = T.time_ms / 1000.0;

u_y = T.y_voltage - Y_NEUTRAL;
u_x = T.x_voltage - X_NEUTRAL;

y_v = T.v / SCALE_DIV;
y_w = T.w / SCALE_DIV;

valid = [true; diff(t) > 0];
t   = t(valid);
u_y = u_y(valid);
u_x = u_x(valid);
y_v = y_v(valid);
y_w = y_w(valid);

Ts = mean(diff(t));

U = [u_y, u_x];
Y = [y_v, y_w];

data_mimo_combined = iddata(Y, U, Ts, ...
    'InputName',  {'delta_y_voltage','delta_x_voltage'}, ...
    'OutputName', {'linear_velocity','angular_velocity'}, ...
    'TimeUnit',   'seconds');

data_mimo_combined_d = detrend(data_mimo_combined);

save('cmd_vel_combined_iddata.mat', ...
    'data_mimo_combined', 'data_mimo_combined_d', ...
    'Ts', 't', 'U', 'Y');

fprintf('Samples: %d\n', size(Y,1));
fprintf('Ts: %.6f sec\n', Ts);
disp('Saved: cmd_vel_combined_iddata.mat');