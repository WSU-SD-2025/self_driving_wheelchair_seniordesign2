%%
% 1) X voltage -> vL, vR
%%
clear; clc;

SPEED_SCALE_DIV = 6.218;

Tang = readtable('/home/sejunmoon/self_driving_wheelchair/log/wheelchair_log_angular_ground.csv');

% time
t = (Tang.time_ms - Tang.time_ms(1)) / 1000;
Ts = mean(diff(t));

% input
u = Tang.x_voltage;

% output
x_vL = Tang.vL / SPEED_SCALE_DIV;
x_vR = Tang.vR / SPEED_SCALE_DIV;

% iddata for each wheel
data_x_vL = iddata(x_vL, u, Ts);
data_x_vR = iddata(x_vR, u, Ts);
