close all;
clc;
clear;

addpath("C:\Program Files\MATLAB\Casadi")
addpath("src")

% system variables
Ts = 1/20; % sampling time
Tf = 4.0; % Time to simulate for
rocket = Rocket(Ts);

[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)
