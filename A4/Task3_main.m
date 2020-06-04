close all
clear all
clc

Trajectory='B';



%% Setup Simulation Environment
syms t
up = 0; % manual elbow position

% initial state
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


% Joint Limis 
    Q1_min = -60*2*pi/360;
    Q1_max = 60*2*pi/360;
    Q2_min = -90*2*pi/360;
    Q2_max = 90*2*pi/360;


mu = 0.1; % Friction coefficient

% Generate dynamics
[A,B,C,G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy);

% % 
% % %% Trajectory A simulation
X0 = [0.710; 0]; Y0 = [1.080; 0]; % initial state
Xf = [1.485; 0]; Yf = [0.041; 0]; % final state
tf = 5; % finish time

if Trajectory=='A'
[Xref, Yref] = trajectoryGen(X0, Y0, Xf, Yf, tf, 1);


% Generate joint space trajectories for the robot and simulator
[Q1_ref, Q2_ref] = inverseKinematics(Xref(t), Yref(t), L_1, L_2, up);

% initial joint space conditions
Q1_0=Q1_ref(0);
Q2_0=Q2_ref(0);

Q=[Q1_0',Q2_0'];

% Create state and robot for simulation
sim_state = State(Q, A, B, C, G);
robot = Robot(Q, mdl_err);
hacker = Robot(Q, mdl_err); % cheats at the game for debug reasons

% Simulation loop
for time = 0:dt_pid:tf
    
    X0 = sim_state.X;
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    % calculate current refernce velocity in joint space
    
    q1_ref = Q1_ref(time);
    q2_ref = Q2_ref(time);
    
    % cheats
    [hacker,~] = hacker.HAX([q1_ref(1), q2_ref(1),q1_ref(2), q2_ref(2)],time, dt_pid);
    
    U = [q1_ref(2), q2_ref(2)]; % ref
    V = [X0(3), X0(4)];     % current velocity

    % Give the robot current velocity "measurement" and reference
    robot = robot.sample(V, U);
    
    
    % Get new command from robot every dt
    if mod(time, dt) == 0
        time;
        [robot,tau] = robot.PID(U,time,dt);
        
    end
   
    
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G, tau, mu), [0, dt_pid], X0);
    lX = length(X);
    
    % Enforse joint limits
    Xnext = X(lX,:);
    % Joint 1
    if Xnext(1) < Q1_min
        Xnext(1) = Q1_min;
    elseif Xnext(1) > Q1_max
        Xnext(1) = Q1_max;
    end
    % Joint 2
    if Xnext(2) < Q2_min
        Xnext(2) = Q2_min;
        
    elseif Xnext(2) > Q2_max
        Xnext(2) = Q2_max;
    end
    
    % update state
    sim_state = sim_state.updateState(Xnext, time+dt_pid);
end

% % %% Misc plotting and animation
% % 
% % hacker.state.plotQ(tf);
% % title('Reference joint angles');
% % figure()
% % sim_state.plotQ(tf);
% % title('True joint angles');
% % 
% % robot.plotError()
% % 
% % 
% % robot.plotTau(1)
% %  
hacker.state.animate(dt, tf, L_1, L_2,10);
% % % sim_state.animate(dt, tf, L_1, L_2,10);
% % 
% % %% Task 3.1 & 3.2 Plots
% % 
% % 

else
%% Trajectory B Simulation

up=0;
% Generate dynamics
[A,B,C,G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy);

X0 = [0.71; 0]; Y0 = [1.08; 0]; % initial state
Xf = [1.36; 0]; Yf = [-0.59; 0]; % final state
tf = 5; % finish time
[Xref, Yref] = trajectoryGen(X0, Y0, Xf, Yf, tf, 1);


% Generate joint space trajectories for the robot and simulator
[Q1_ref, Q2_ref] = inverseKinematics(Xref(t), Yref(t), L_1, L_2, up);

% initial joint space conditions
Q1_0=Q1_ref(0);
Q2_0=Q2_ref(0);

Q=[Q1_0',Q2_0'];

% Create state and robot for simulation
sim_state = State(Q, A, B, C, G);
robot = Robot(Q, mdl_err);
hacker = Robot(Q, mdl_err); % cheats at the game for debug reasons

% Simulation loop
for time = 0:dt_pid:tf
    
    X0 = sim_state.X;
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    % calculate current refernce velocity in joint space
    
    q1_ref = Q1_ref(time);
    q2_ref = Q2_ref(time);
    
    % cheats
    [hacker,~] = hacker.HAX([q1_ref(1), q2_ref(1),q1_ref(2), q2_ref(2)],time, dt_pid);
    
    U = [q1_ref(2), q2_ref(2)]; % ref
    V = [X0(3), X0(4)];     % current velocity

    % Give the robot current velocity "measurement" and reference
    robot = robot.sample(V, U);
    
    
    % Get new command from robot every dt
    if mod(time, dt) == 0
        time;
        [robot,tau] = robot.PID(U,time,dt);
        
    end
   
    
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G, tau, mu), [0, dt_pid], X0);
    lX = length(X);
    
    % Enforse joint limits
    Xnext = X(lX,:);
    % Joint 1
    if Xnext(1) < Q1_min
        Xnext(1) = Q1_min;
    elseif Xnext(1) > Q1_max
        Xnext(1) = Q1_max;
    end
    % Joint 2
    if Xnext(2) < Q2_min
        Xnext(2) = Q2_min;
        
    elseif Xnext(2) > Q2_max
        Xnext(2) = Q2_max;
    end
    
    % update state
    sim_state = sim_state.updateState(Xnext, time+dt_pid);
end


%% Misc plotting and animation



% hacker.state.plotQ(tf);
% title('Reference joint angles');
% figure()
sim_state.plotQ(tf);
% title('True joint angles');
% 
% robot.plotError()
% 
% 
% robot.plotTau(1)
 
hacker.state.animate(dt, tf, L_1, L_2,10);
% sim_state.animate(dt, tf, L_1, L_2,10);



%% Task 3.3 Plots

end

