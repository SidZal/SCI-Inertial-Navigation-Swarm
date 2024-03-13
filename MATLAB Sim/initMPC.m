
import casadi.*

wL_max = w_max;
wL_min = -w_max;
wR_max = w_max;
wR_min = -w_max;


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
    st_new = st_old + dt*f_value;
    X(:,k+1) = st_new;
end

obj = 0; % Objective function

Q = diag([1,1,0.1]);
R = diag([0,0]);

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