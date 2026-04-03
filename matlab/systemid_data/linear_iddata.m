clc; clear; close all;

T = readtable("/home/sejunmoon/self_driving_wheelchair/log/angular_desk.csv");

time = T.time_ms / 1000;
Ts = mean(diff(time));

x_v = T.x_voltage;

xL = T.vL;
xR = T.vR;

dataL = iddata(xL, x_v, Ts);
dataR = iddata(xR, x_v, Ts);

