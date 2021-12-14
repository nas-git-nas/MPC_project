close all;
clc;
clear;

addpath("C:\Program Files\MATLAB\Casadi")
addpath("src")

% system variables
Ts = 1/20; % sampling time
Tf = 4.0; % Time to simulate for
rocket = Rocket(Ts);

% input variables
d1 = 0; % deflection angles servo 1, [-0.26rad, 0.26rad]
d2 = 0; % deflection angles servo2,  [-0.26rad, 0.26rad]
Pavg = 56.66; % average throttle, [20%, 80%]
Pdiff = 0; % throttle difference, [-20%, 20%]

% state variables
wx = 0; % angular velocity around body x axis [rad/s]
wy = 0; % angular velocity around body y axis [rad/s]
wz = 0; % angular velocity around body z axis [rad/s]
alpha = 0; % euler angle around body x axis [rad]
beta = 0; % euler angle around body y axis [rad]
gamma = 0; % euler angle around body z axis [rad]
vx = 0; % velocity in world frame in x direction [m/s]
vy = 0; % velocity in world frame in y direction [m/s]
vz = 0; % velocity in world frame in z direction [m/s]
px = 0; % position in world frame in x direction [m]
py = 0; % position in world frame in y direction [m]
pz = 0; % position in world frame in z direction [m]

% input vector
u = [d1 d2 Pavg Pdiff]';

% state vector
w = [wx wy wz]';
phi = [alpha beta gamma]';
v = [vx vy vz]';
p = [px py pz]';
x0 = [w' phi' v' p']';

% % exercise 1.1
% rocket = Rocket(Ts);
% [b_F, b_M] = rocket.getForceAndMomentFromThrust(u);
% x_dot = rocket.f(x0, u);

% exercise 1.2
[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U); % Trajectory visualization at 1.0x real−time
