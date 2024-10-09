
% Loading the equations of motion
load('DesignProblem03_EOMs.mat');

% Defining Variables
f = symEOM.f;
syms theta phi xdot ydot thetadot phidot;

%Linearizing%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definining s vector with all EOM variables
w = [theta; phi; xdot; ydot; thetadot; phidot];
% Define state vector x
sX = [theta; phi; xdot; ydot; thetadot];
% Define d/dt of x,y,theta from parsed EOM
xddot = f(1);
yddot = f(2);
thetaddot = f(3);
% Define d/dt of state vector x
sXdot = [thetadot; phidot; xddot; yddot; thetaddot];

% % Make theta_e substitution in f
% theta_e = .1*pi/180;
% f1 = subs(f,[theta],[theta_e]);

v = subs(sXdot, [thetadot; phidot], [0;0]);

% Define input vector u
u = [phidot];
%Define output vector y
y = [theta;phi];
% Use matlabFunction to convert symbolic expression of f1 to function handle
% using EOM variables
f_numeric = matlabFunction(v,'vars',{w});
% Define w guess
% Where w = [theta; phi; xdot; ydot; thetadot; phidot];
w_guess = [0.01; 0; 7; -.1; 0; 0];
% Evaulate f_numeric at w_guess in fsolve
[w_sol, f_numeric_at_w_sol, exitflag] = fsolve(f_numeric, w_guess,...
    optimoptions(@fsolve,'Algorithm','levenberg-marquardt','Display','off'));
% Define w equilbrium vector from w_sol
w_e = w_sol


A = double(subs(jacobian(sXdot,sX),w,w_e));
B = double(subs(jacobian(sXdot,u),w,w_e));
C = double(subs(jacobian(y,sX),w,w_e));
D = double(subs(jacobian(y,u),w,w_e));


% controlability matrix
W = ctrb(A,B);
isControlable = logical(length(A) == rank(W));

% Check A for stability - all negative real eigenvalues
stabA = eig(A);

% Define observability matrix
Ob = obsv(A,C);
isObservable = logical(length(A) == rank(Ob));

%Finding K
Qc = 1.8*[200 0 0 0 0; 0 1 0 0 0; 0 0 10 0 0; 0 0 0 10 0; 0 0 0 0 150];
Rc = 0.1*[1];
K = lqr(A,B,Qc,Rc);

%Finding L
Q0 = 0.2*[1 0;0 1];
R0 = 0.8*[1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1];
L = lqr(A',C',R0^(-1),Q0^(-1))'

%Checking aymptotic stability
stabABK =eig(A-B*K)
stabALC =eig(A-L*C)

%Kref for reference tracking
kRef = 1/(C*inv(A-B*K)*B)
