close all
clear all
clc



%% Setup Simulation Environment

up = 0; % manual elbow position

% initial state
X0 = 2*pi/360.*[30, 75, 0, 0];
dt = 0.01;
dt_pid = 0.001;
tf = 5;
mdl_err = 0; % robot modeling error


% True Physical Parameters
m_1 = 2; %kg
m_2 = 1; %kg
L_1 = 1; %m
L_2 = 0.6; %m
r_c1 = L_1/2; %m
r_c2 = L_2/2; %m

I_zz1 = 0.5; %kgm^2
I_zz2 = 0.3; %kgm^2
I_xx = 0; %kgm^2
I_yy = 0; %kgm^2

mu = 0.1; % Friction coefficient

% Generate dynamics
[A,B,C,G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy);


%% Trajectory A
X0 = [0.710; 0]; Y0 = [1.080; 0]; % initial state
Xf = [1.485; 0]; Yf = [0.041; 0]; % final state
tf = 5; % finish time

% Create state and robot for simulation
sim_state = State([X0', Y0'], A, B, C, G);
robot = Robot([X0', Y0'], mdl_err);
hacker = Robot([X0', Y0'], mdl_err);

[X, Y] = trajectoryGen(X0, Y0, Xf, Yf, tf, 0);

[Q1,Q2] = Task3_dynamics(X, Y, L_1, L_2, up);




% Simulate
for time = 0:dt_pid:tf
   
    X0 = sim_state.X;
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    

    
    % calculate refernce velocity
    RdQ1=Q1(time);
    RdQ2=Q2(time);
    
        % update hacker
    X_ref = [RdQ1(1),RdQ1(2), RdQ2(1), RdQ2(2)];
    [hacker,~] = hacker.HAX(X_ref,time, dt_pid);
    
    U = [RdQ1(2), RdQ2(2)];
    
    V = [X0(3), X0(4)];

    % Give the robot current velocity "measurement"
    robot = robot.sample(V, U);
    
    
    % Get new command from robot every dt
    if mod(time, dt) == 0
        time;
        [robot,tau] = robot.PID(U,time,dt);
        
    end
   
    
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G, tau, mu), [0, dt_pid], X0);
    lX = length(X);
    sim_state = sim_state.updateState(X(lX,:), time+dt_pid);

end

% hacker.state.animate(dt, tf, L_1, L_2,10);
hacker.state.plotQ(tf);
% sim_state.animate(dt, tf, L_1, L_2,10);
% 
robot.plotError()
% robot.plotError2()
robot.plotTau(1)
% 












