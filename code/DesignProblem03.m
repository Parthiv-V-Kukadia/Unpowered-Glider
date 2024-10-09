function DesignProblem03(controller,varargin)
% DesignProblem03   run simulation of glider
%
%   DesignProblem03('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%   DesignProblem03('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'team' : a name (e.g., 'FirstName LastName') that, if defined,
%                    will appear on the figure window
%
%           'datafile' : a filename (e.g., 'data.mat') where, if defined,
%                        data will be logged and saved
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'controllerdatatolog' : a cell array (e.g., {'y','xhat'}) with
%                                   the names of fields in controller.data
%                                   - if 'datafile' is defined (so data is
%                                   logged), then values in these fields
%                                   will also be logged and saved
%
%           'tStop' : the time at which the simulation will stop (a
%                     positive number) - default value is 30
%
%           'reference' : a function of time that specifies a reference
%                         value theta - for example, the following choice
%                         would specify theta(t) = sin(t):
%                           @(t) sin(t)
%
%           'initial' : a 7x1 matrix
%                           [x y theta phi xdot ydot thetadot]
%                       that specifies initial values - by default, these
%                       values are sampled from a multivariate normal
%                       distribution
%
%           'launchangle' : an angle in radians that will be the mean of
%                           the normal distributions from which the initial
%                           pitch angle is sampled, unless this angle is
%                           specified by the optional argument 'initial'
%                           (the initial velocity vector will be aligned
%                           with the pitch angle as well, more or less) -
%                           default value is 0
%
%           'elevatorlen': a number between 0 and 0.2 that specifies the 
%                          length of the elevator. Takes a default value of
%                          0.05                          
%
%           'display' : a flag...
%
%                       - If true, it will clear the current figure and
%                         will show the simulation. To quite, type 'q' when
%                         this figure is in the foreground.
%
%                       - If false, it will not show any graphics and will
%                         run the simulation as fast as possible (not in
%                         real-time).
%
%           'seed' : a non-negative integer that, if defined, will be used
%                    to seed the random number generator - by fixing the
%                    seed, you make your simulation results the same every
%                    time - this is often useful when testing

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'controller',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'team',[],@ischar);
addParameter(p,'datafile',[],@ischar);
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'controllerdatatolog',[],@iscell);
addParameter(p,'tStop',30,@(x) isscalar(x) && isnumeric(x) && (x>0));
addParameter(p,'reference',@(x)0,@(x) isa(x,'function_handle'));
addParameter(p,'launchangle',0,@(x) isscalar(x) && isnumeric(x));
addParameter(p,'elevatorlen',0,@(x) isscalar(x) && isnumeric(x) && (x>0) && (x<=0.2));
addParameter(p,'initial',[],@(x) validateattributes(x,{'numeric'},{'size',[7 1]}));
addParameter(p,'display',true,@islogical);
addParameter(p,'seed',[],@(x) isnumeric(x) && (x>=0) && (x<=2^32));
% - Apply input parser
parse(p,controller,varargin{:});
% - Extract parameters
process = p.Results;
% - Check that the 'controller' function exists
if (exist(process.controller,'file')~=2)
    error('Controller ''%s'' does not exist.',process.controller);
end

% Setup the simulation
[process,controller] = SetupSimulation(process);

% Run the simulation
RunSimulation(process,controller);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)

% SEED RANDOM NUMBER GENERATOR

if isempty(process.seed)
    rng('shuffle');
else
    rng(process.seed);
end

% DEFINE CONSTANTS

% Constants related to simulation.
% - State time.
process.tStart = 0;
% - Time step.
process.tStep = 1/100;
% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','x','y','theta','phi','xdot','ydot','thetadot'};

% Constants related to physical properties.
% - Mass
process.m = 0.05;
% - Gravity
process.g = 9.81;
% - Density of air
process.rho = 1.292;
% - Lengths
process.l = 0.35;
process.lw = -0.03;
if process.elevatorlen == 0
    process.le = 0.05;
else
    process.le = process.elevatorlen;
end
% - Area of wing and tail (elevator) control surfaces
process.Sw = 0.1;
process.Se = 0.5*process.le;
% - Moment of inertia
process.J = 6e-3;
    
% - Dimensions (for display only)
process.dWing = [-0.1 0.1 -0.0075 0.0075];
process.dElev = [-process.le 0 -0.0075 0.0075];
process.dSpar = [-process.l 0 -0.0025 0.0025];

% - EOM
filename = 'DesignProblem03_EOMs.mat';
if (exist(filename,'file')==2)
    load(filename);
else
    [symEOM,numEOM] = GetEOM(process.m,process.g,process.J,...
                             process.rho,...
                             process.Sw,process.Se,...
                             process.l,process.lw,process.le);
	fprintf(1,'Saving EOMs to file (load %s to work with them).\n',filename);
	save('DesignProblem03_EOMs.mat','symEOM','numEOM');
end
process.symEOM = symEOM;
process.numEOM = numEOM;
% - Maximum elevator rate
process.phidotMax = 1;

% DEFINE VARIABLES

% Time
process.t = 0;
% States
if isempty(process.initial)
    theta = process.launchangle;
    a = [0; 2; theta; 0; 6 * cos(theta); 6 * sin(theta); 0];
    b = [0; 0; 0.05*pi; 0.05*pi; 0.1; 0.1; 0.05];
    process.initial = a + b.*randn(7, 1);
end
process.x = process.initial(1,1);
process.y = process.initial(2,1);
process.theta = process.initial(3,1);
process.phi = process.initial(4,1);
process.xdot = process.initial(5,1);
process.ydot = process.initial(6,1);
process.thetadot = process.initial(7,1);

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','phidotMax','symEOM','numEOM'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% Status
controller.running = true;
% Init
tic
try
    [controller.data] = ...
        controller.init(controller.parameters, ...
                        controller.data);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tInit = toc;
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
controller = RunController(controller);
end

function controller = RunController(controller)
if (controller.running)
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.run(controller.sensors, ...
                              controller.references, ...
                              controller.parameters, ...
                              controller.data);
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tRun = toc;
else
    controller.tRun = 0;
end
end

function references = GetReferences(process)
try
    references = struct('theta',process.reference(process.t));
catch exception
    warning(['The ''references'' function that was passed to\n' ...
             'DesignProblem03 threw the following error:\n\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Leaving references unchanged.\n'],getReport(exception));
    references = struct('theta',0);
end
end

function [symEOM,numEOM] = GetEOM(m,g,J,rho,Sw,Se,l,lw,le)
% States
syms x y theta phi xdot ydot thetadot real
% Inputs
syms phidot real
% EOMs
xwdot = [xdot+lw*thetadot*sin(theta);
         ydot-lw*thetadot*cos(theta)];
xedot = [xdot+l*thetadot*sin(theta)+le*(thetadot+phidot)*sin(theta+phi);
         ydot-l*thetadot*cos(theta)-le*(thetadot+phidot)*cos(theta+phi)];
aw = theta-atan2(xwdot(2),xwdot(1));
ae = theta+phi-atan2(xedot(2),xedot(1));
fw = rho*Sw*(xwdot(1)^2+xwdot(2)^2)*sin(aw);
fe = rho*Se*(xedot(1)^2+xedot(2)^2)*sin(ae);
xdd = (1/m)*(-fw*sin(theta)-fe*sin(theta+phi));
ydd = (1/m)*(fw*cos(theta)+fe*cos(theta+phi)-m*g);
thetadd = (1/J)*(-fw*lw-fe*(l*cos(phi)+le));
% Symbolic
symEOM.f = [xdd; ydd; thetadd];
% Numeric
numEOM.f = matlabFunction(symEOM.f,'Vars',[theta,phi,xdot,ydot,thetadot,phidot]);
end

function sensors = GetSensors(process)
sensors.t = process.t;
sensors.theta = process.theta;
sensors.phi = process.phi;
% Add noise
%   (nothing)
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.x; process.y; process.theta; process.phi; process.xdot; process.ydot; process.thetadot];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = actuators.phidot;

% Bound input
if (u < -process.phidotMax)
    u = -process.phidotMax;
elseif (u > process.phidotMax)
    u = process.phidotMax;
end

% Add disturbance
%   (nothing)
end

function process = Get_Process_From_TandX(t,x,process)
process.t = t;
process.x = x(1,1);
process.y = x(2,1);
process.theta = x(3,1);
process.phi = x(4,1);
process.xdot = x(5,1);
process.ydot = x(6,1);
process.thetadot = x(7,1);
end

function xdot = GetXDot(t,x,u,process)
theta = x(3,1);
phi = x(4,1);
xdot = x(5,1);
ydot = x(6,1);
thetadot = x(7,1);
phidot = u(1,1);
xdot = [xdot; ydot; thetadot; phidot; process.numEOM.f(theta,phi,xdot,ydot,thetadot,phidot)];
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if all(isfield(actuators,{'phidot'}))&&(length(fieldnames(actuators))==1)
    if isnumeric(actuators.phidot)
        if isscalar(actuators.phidot)
            iscorrect = true;
        end
    end
end
end

function actuators = ZeroActuators()
actuators = struct('phidot',0);
end

function res = ShouldStop(process)
if process.y <= 0
    res = true;
else
    res = false;
end
end

function fig = UpdateFigure(process,controller,fig)
if (isempty(fig))
    % CREATE FIGURE

    % Clear the current figure.
    clf;

    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    fig.text.status=text(0.05,0.975,...
        sprintf('CONTROLLER: %s',status),...
        'fontweight','bold','fontsize',fs,...
        'color',color,'verticalalignment','top');
    fig.text.time=text(0.05,0.12,...
        sprintf('flight time: %6.2f\n',process.t),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,...
        sprintf('%s',process.team),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');
    fig.text.x=text(0.95,0.12,...
        sprintf('flight distance: %6.2f',process.x),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco','horizontalalignment','right');
    
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    
    fig.view0.dx = 1.1;
    fig.view0.dy = 0.5;
    
    xmin = 0;
    xmax = 40;
    xstep = 1;
    ymin = 0;
    ymax = 3;
    ystep = 1;
    
    fig.view0.axis = axes('position',[0 0 1 0.8]);
    axis equal;
    set(fig.view0.axis,'xlim',[process.x-fig.view0.dx process.x+fig.view0.dx]);
    set(fig.view0.axis,'ylim',[process.y-fig.view0.dy process.y+fig.view0.dy]);
    axis manual;
    hold on;
    axis off;
    box on;
    
    for x = xmin:xstep:xmax
        line([x x],[ymin ymax],'color',0.9*[1 1 1],'linewidth',2);
    end
    for y = ymin+ystep:ystep:ymax
        line([xmin xmax],[y y],'color',0.9*[1 1 1],'linewidth',2);
    end
    line([xmin xmax],[0 0],'color',0.5*[1 1 1],'linewidth',4);

    fig.view0.trace = plot(process.x,process.y,'r-');
    xe = process.x-process.l*cos(process.theta);
    ye = process.y-process.l*sin(process.theta);
    fig.view0.spar = DrawBox([],process.x,process.y,process.theta,process.dSpar);
    fig.view0.wing = DrawBox([],process.x,process.y,process.theta,process.dWing);
    fig.view0.elev = DrawBox([],xe,ye,process.theta+process.phi,process.dElev);
    
    fig.view1.axis = axes('position',[0.05 0.75 0.9 0.2]);
    axis equal;
    axis([xmin xmax ymin ymax]);
    hold on;
    grid on;
    axis manual;
    hold on;
    axis off;
    
    for x = xmin:xstep:xmax
        line([x x],[ymin ymax],'color',0.9*[1 1 1],'linewidth',1);
    end
    for y = ymin+ystep:ystep:ymax
        line([xmin xmax],[y y],'color',0.9*[1 1 1],'linewidth',1);
    end
    line([xmin xmax],[0 0],'color',0.5*[1 1 1],'linewidth',2);
    
    fig.view1.trace = plot(process.x,process.y,'r-');
    fig.view1.spar = DrawBox([],process.x,process.y,process.theta,process.dSpar);
    fig.view1.wing = DrawBox([],process.x,process.y,process.theta,process.dWing);
    fig.view1.elev = DrawBox([],xe,ye,process.theta+process.phi,process.dElev);

    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
end

% UPDATE FIGURE

set(fig.text.time,'string',sprintf('flight time: %6.2f\n',process.t));
set(fig.text.x,'string',sprintf('flight distance: %6.2f\n',process.x));
if (controller.running)
    status = 'ON';
    color = 'g';
else
    status = 'OFF';
    color = 'r';
end
set(fig.text.status,'string',sprintf('CONTROLLER: %s',status),'color',color);

set(fig.view0.axis,'xlim',[process.x-fig.view0.dx process.x+fig.view0.dx]);
set(fig.view0.axis,'ylim',[process.y-fig.view0.dy process.y+fig.view0.dy]);

xe = process.x-process.l*cos(process.theta);
ye = process.y-process.l*sin(process.theta);
fig.view0.spar = DrawBox(fig.view0.spar,...
                         process.x,process.y,process.theta,...
                         process.dSpar);
fig.view0.wing = DrawBox(fig.view0.wing,...
                         process.x,process.y,process.theta,...
                         process.dWing);
fig.view0.elev = DrawBox(fig.view0.elev,...
                         xe,ye,process.theta+process.phi,...
                         process.dElev);
x = [get(fig.view0.trace,'xdata') process.x];
y = [get(fig.view0.trace,'ydata') process.y];
set(fig.view0.trace,'xdata',x,'ydata',y);

fig.view1.spar = DrawBox(fig.view1.spar,...
                         process.x,process.y,process.theta,...
                         process.dSpar);
fig.view1.wing = DrawBox(fig.view1.wing,...
                         process.x,process.y,process.theta,...
                         process.dWing);
fig.view1.elev = DrawBox(fig.view1.elev,...
                         xe,ye,process.theta+process.phi,...
                         process.dElev);
set(fig.view1.trace,'xdata',x,'ydata',y);

drawnow;
end

function box = DrawBox(box,x,y,theta,d)
R = [cos(theta) -sin(theta);
     sin(theta) cos(theta)];
o = [x;y];
p = [d(1) d(2) d(2) d(1) d(1);
     d(3) d(3) d(4) d(4) d(3)];
p = repmat(o,1,size(p,2))+R*p;
if isempty(box)
    box = fill(p(1,:),p(2,:),'y');
else
    set(box,'xdata',p(1,:),'ydata',p(2,:));
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(process.moviefile))
    myV = VideoWriter(process.moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/process.tStep;
    open(myV);
end

% LOOP

% Loop until break.
tStart = tic;
while (1)

    % Update figure (create one if fig is empty).
    if (process.display)
        fig = UpdateFigure(process,controller,fig);
    end

    % Update data.
    if (~isempty(process.datafile))
        [process,controller] = UpdateDatalog(process,controller);
    end

    % If making a movie, store the current figure as a frame.
    if (~isempty(process.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end

    % Stop if time has reached its maximum.
    if ((process.t + eps >= process.tStop)||done||ShouldStop(process))
        break;
    end

    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);

    % Wait if necessary, to stay real-time.
    if (process.display)
        while (toc(tStart)<process.t-process.tStart)
            % Do nothing
        end
    end

end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(process.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
if (~isempty(process.datafile))
    processdata = process.log.process; %#ok<NASGU>
    controllerdata = process.log.controller; %#ok<NASGU>
    save(process.datafile,'processdata','controllerdata');
end

% Save the snapshot, if necessary.
if (~isempty(process.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',process.snapshotfile);
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tInit',[],...
                                             'tRun',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write process data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
% Write controller data to log, if controller is running.
if controller.running
    process.log.controller.tInit = controller.tInit;
    process.log.controller.tRun(:,process.log.count) = ...
        controller.tRun;
    names = fieldnames(controller.sensors);
    for i=1:length(names)
        name = names{i};
        process.log.controller.sensors.(name)(:,process.log.count) = ...
            controller.sensors.(name);
    end
    names = fieldnames(controller.actuators);
    for i=1:length(names)
        name = names{i};
        process.log.controller.actuators.(name)(:,process.log.count) = ...
            controller.actuators.(name);
    end
    for i=1:length(process.controllerdatatolog)
        name = process.controllerdatatolog{i};
        try
            process.log.controller.data.(name)(:,process.log.count) = ...
                controller.data.(name);
        catch exception
            warning(['Saving element ''%s'' of data for controller\n',...
                     '     ''%s''',...
                     'threw the following error:\n\n' ...
                     '==========================\n' ...
                     '%s\n', ...
                     '==========================\n\n' ...
                     'Turning off controller and setting all\n' ...
                     'actuator values to zero.\n'],...
                     name,controller.name,getReport(exception));
            controller.actuators = ZeroActuators();
            controller.running = false;
            return
        end
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,x] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
[t,x] = ode45(@(t,x) GetXDot(t,x,u,process),[t0 t0+process.tStep],x);
process = Get_Process_From_TandX(t(end),x(end,:)',process);
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
controller = RunController(controller);
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
