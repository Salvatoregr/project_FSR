%% NMPC init
%Load this script before running the simulation

run("LOAD_NMPC.m")

for i=1:length(PP)-1
    PSI(1,i)=atan2(PP(2,i+1)-PP(2,i),PP(1,i+1)-PP(1,i));
end
PSI(1,4001)=PSI(1,4000);

for i=579:609
    PSI(1,i)=PSI(1,i)-2*pi;
end

for i=1177:4001
    PSI(1,i)=PSI(1,i)+2*pi;
end

for i=1914:1943
    PSI(1,i)=PSI(1,i)-2*pi;
end

for i=2510:4001
    PSI(1,i)=PSI(1,i)+2*pi;
end

for i=3247:3277
    PSI(1,i)=PSI(1,i)-2*pi;
end

for i=3842:4001
    PSI(1,i)=PSI(1,i)+2*pi;
end

%% PARAMS

m=0.85;
mass=0.85;
l=0.15;
J=diag([2.5 2.1 4.3])*10^-3;
c=0.022;
g=9.81;


%% NMPC Parameters

my_NMPC=nlmpc(12,4,4);
Ts=0.1;

my_NMPC.Ts = Ts;
my_NMPC.PredictionHorizon = 20;
my_NMPC.ControlHorizon = 5;

%% 

my_NMPC.Model.StateFcn = 'NMPC_XFcn';
my_NMPC.Model.OutputFcn = 'NMPC_YFcn';

my_NMPC.MV(1).Min = 0;
my_NMPC.MV(2).Min = 0;
my_NMPC.MV(3).Min = 0;
my_NMPC.MV(4).Min = 0;

my_NMPC.MV(1).ScaleFactor = 1;
my_NMPC.MV(2).ScaleFactor = 1;
my_NMPC.MV(3).ScaleFactor = 1;
my_NMPC.MV(4).ScaleFactor = 1;

my_NMPC.States(1).ScaleFactor = 1;
my_NMPC.States(2).ScaleFactor = 1;
my_NMPC.States(3).ScaleFactor = 1;
my_NMPC.States(4).ScaleFactor = 1;
my_NMPC.States(5).ScaleFactor = 1;
my_NMPC.States(6).ScaleFactor = 1;
my_NMPC.States(7).ScaleFactor = 1;
my_NMPC.States(8).ScaleFactor = 1;
my_NMPC.States(9).ScaleFactor = 1;
my_NMPC.States(10).ScaleFactor = 1;
my_NMPC.States(11).ScaleFactor = 1;
my_NMPC.States(12).ScaleFactor = 1;

my_NMPC.Weights.OutputVariables = [10 10 10 10]; %BESTs: (1)[10 10 40] (2)[20 20 10] (3)[10 10 10]

% For the tuning, start from a BEST configuration and vary the combinations. 
% The trends of X and Y are the same, while Z exhibits different behavior.
% X and Y exhibit generally more delay in tracking the reference.
% For calibration I am using the sine wave because in theory if it works there it can also work on the trajectory.
% The simulations take forever but it is inevitable.

% As future developments, possibility of similarly implementing a simple MPC could be investigated; MATLAB should offer a toolbox
% for automatic calibration analogous to PIDs.

% BEST UNTIL NOW: TS=0.01 PH=30 CH=1 [100 100 30] Follows with delay = 0.3 s.
% BEST UNTIL NOW: TS=0.1 PH=20 CH=5 [10 10 100] Follows trajectory without screwing up but badly.

% WINNER: TS=0.1 PH=20 CH=5 [10 10 10] 40s


