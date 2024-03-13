clear;clc;close all
%% Library
import casadi.*

%% Physical parameters
r = 0.036; 
d = 0.149; 

%% MPC parameters
T = 0.1; % Sampling time [s]
N = 20; % Prediction horizon

wL_max = 10;
wL_min = -wL_max;
wR_max = 10;
wR_min = -wR_max;

%% MPC definition
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');

states = [x;y;theta];
n_states = length(states);
wL = SX.sym('wL');
wR = SX.sym('wR');

controls = [wL;wR];
n_controls = length(controls);
rhs = [r/2*(wL+wR)*cos(theta);
       r/2*(wL+wR)*sin(theta);
       r/d*(wR-wL)];

% v = SX.sym('v');
% w = SX.sym('w');
% controls = [v;w];
% n_controls = length(controls);
% rhs = [v*cos(theta);
%        v*sin(theta);
%        w];

f = Function('f', {states,controls}, {rhs}); % Nonlinear mapping function f(x,u)
U = SX.sym('U', n_controls, N); % Decesion variables (controls)
P = SX.sym('P', n_states + n_states);
X = SX.sym('X', n_states, N+1);

% Compute solution symbolically
X(:,1) = P(1:3); % Initial state
for k = 1:N
    st_old = X(:,k);
    con_old = U(:,k);
    f_value = f(st_old,con_old);
    st_new = st_old + T*f_value;
    X(:,k+1) = st_new;
end

ff = Function('ff', {U,P}, {X});

obj = 0; % Objective function

Q = diag([1,1,0.5]);
R = diag([0.,0.]);

% Compute objective
for k = 1:N
    st = X(:,k);
    con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6))+con'*R*con;
end

% Make the decesion variables one column vector
OPT_variables = reshape(U, 2*N, 1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

% Input constraints
args = struct;
args.lbx(1:2:2*N,1) = wL_min;
args.lbx(2:2:2*N,1) = wR_min;
args.ubx(1:2:2*N,1) = wL_max;
args.ubx(2:2:2*N,1) = wR_max;

robot = dualWheelRobot(T,1,"r");

% Simulation loop
t0 = 0;
x0 = [0;0;0]; % Initial condition
xs = [6;-3;0]; % Reference posture
xx(:,1) = x0; % State history
t(1) = t0;
u0 = zeros(N,2); % Control inputs
sim_tim = 30; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_c1 = [];

while mpciter < sim_tim/T

    args.p = [x0;xs];
    args.x0 = reshape(u0',2*N,1);
    sol = solver('x0',args.x0,'lbx',args.lbx,'ubx',args.ubx,'p',args.p);
    u = reshape(full(sol.x)', 2, N)';

    ff_values = ff(u',args.p);
    xx1(:,1:3,mpciter+1) = full(ff_values');
    u_c1 = [u_c1; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u, f);
    xx(:,mpciter+2) = x0;
    mpciter = mpciter + 1;
    robot.plotCurrentPos(x0)
    drawnow

end

function [t0, x0, u0] = shift(T, t0, x0, u, f)
st = x0;
con = u(1,:)';
f_value = f(st,con);
st = st + T*f_value;
x0 = full(st);
t0 = t0 + T;
u0 = [u(2:size(u,1),:);
      u(size(u,1),:)];
end
