%%
% 1) Y voltage -> vL, vR
%%
clear; clc;

SPEED_SCALE_DIV = 6.218;

Tlin = readtable('/home/sejunmoon/self_driving_wheelchair/log/wheelchair_log_linear_ground.csv');

% time
t = (Tlin.time_ms - Tlin.time_ms(1)) / 1000;
Ts = mean(diff(t));

% input
u = Tlin.y_voltage;

% output
y_vL = Tlin.vL / SPEED_SCALE_DIV;
y_vR = Tlin.vR / SPEED_SCALE_DIV;

% iddata for each wheel
data_y_vL = iddata(y_vL, u, Ts);
data_y_vR = iddata(y_vR, u, Ts);
