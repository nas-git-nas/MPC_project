close all;
clc;
clear;

addpath("C:\Program Files\MATLAB\Casadi")
addpath("src")
addpath("Deliverable_3_1\")

% define system
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = 10; % Horizon length in seconds
% mpc_x = MPC_Control_x(sys_x, Ts, H);
% mpc_y = MPC_Control_y(sys_y, Ts, H);
% mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Get control input
% ux = mpc_x.get_u(x)

% define simulation variables
Tf = 10.0; % Time to simulate for
x0 = [0; 0; 0; 5];
y0 = [0; 0; 0; 5];
z0 = [0; 5];
roll0 = [0; pi/4];

% simulate controller
% [Tx, X_sub, Ux_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
% [Ty, Y_sub, Uy_sub] = rocket.simulate(sys_y, y0, Tf, @mpc_y.get_u, 0);
% [Tz, Z_sub, Uz_sub] = rocket.simulate(sys_z, z0, Tf, @mpc_z.get_u, 0);
[Troll, ROLL_sub, Uroll_sub] = rocket.simulate(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
% ph = rocket.plotvis_sub(Tx, X_sub, Ux_sub, sys_x, xs, us);
% ph = rocket.plotvis_sub(Ty, Y_sub, Uy_sub, sys_y, xs, us);
% ph = rocket.plotvis_sub(Tz, Z_sub, Uz_sub, sys_z, xs, us);
ph = rocket.plotvis_sub(Troll, ROLL_sub, Uroll_sub, sys_roll, xs, us);
