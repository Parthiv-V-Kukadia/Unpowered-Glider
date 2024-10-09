function func = Controller
% INTERFACE
%
%   sensors
%       .t          (time)
%       .theta      (pitch angle)
%       .phi        (elevator angle)
%
%   references
%       .theta      (reference pitch angle)
%
%   parameters
%       .tStep      (time step)
%       .phidotMax  (maximum elevator angular velocity)  
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .phidot     (elevator angular velocity)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [data] = initControlSystem(parameters, data)


%
data.A = [0 0 0 0 1; 0 0 0 0 0; -6.5711 3.1236 -0.0314 -0.4596 0.2411;191.1751 63.7969 0.9324 -27.3165 3.5291; -207.2970 -239.2626 2.0077 29.4141 -15.4288];
data.B = [0;1;0.0532;0.9070;-3.4018];
data.C = [1 0 0 0 0; 0 1 0 0 0];
data.D = [0];

%Finding K
Qc = 1.8*[200 0 0 0 0; 0 1 0 0 0; 0 0 10 0 0; 0 0 0 10 0; 0 0 0 0 150];
Rc = 0.1*[1];
data.K = lqr(data.A,data.B,Qc,Rc);

%Finding L
Q0 = 0.2*[1 0;0 1];
R0 = 0.8*[1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1];
data.L = lqr(data.A',data.C',inv(R0),inv(Q0))';

%Initial state guesses
data.xdot = 7;
data.zdot = -.1;
data.thetadot = 0;

%state matrix
data.xhat = [0.007 ; 0.01; data.xdot; data.zdot;data.thetadot];
data.xe = [0.0039; -0.0624; 7.0149; -0.4788;0];

%Kref for reference tracking
data.kRef = 1/(data.C*inv(data.A-data.B*data.K)*data.B);
% Here is a good place to initialize things...
%

end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors, references, parameters, data)

data.r = [.05*sensors.theta;0];

t=1/100; % Time-Step

y= [sensors.theta; sensors.phi]- data.C*data.xe;
u= -data.K*data.xhat +data.kRef*data.r;

data.xhat=data.xhat+t*(data.A*data.xhat+data.B*u-data.L*(data.C*data.xhat-y));

actuators.phidot = u;

end
