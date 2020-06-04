close all
clear all
clc

set(0,'defaulttextInterpreter','latex') % LaTex is the only way
%% Robot Dynamics

% 
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


[A,B,C,G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy);



%% Simulate unactuated robot


dt = 0.01;
tf = 5;
mu = 0.1;           % Friction coefficient
tau = [0,0];        % appied joint torques
X0 = [0, 0, 0, 0];  % initial state

sim_state = State(X0, A, B, C, G);


for time = 0:dt:tf
   
    X0 = sim_state.X;
    
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    dynamics=[A,B,C,G];
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G, tau, mu), [0, dt], X0);
    lX = length(X);
    sim_state = sim_state.updateState(X(lX,:), time+dt);

end


sim_state.animate(dt, tf, L_1, L_2,1);
sim_state.plotQ(tf);



%% Robot Vs. Gravity



mdl_err = 0.01;     % modelling error (decimal percent of all params)
dt = 0.01;
tf = 5;
mu = 0.1;           % Friction coefficient
X0 = [0, 0, 0, 0];  % initial state

% The robot has its on state object so it's easy to add error in it's model
% parameters to simulate real world errors neatly.

robot = Robot(X0,mdl_err);
sim_state = sim_state.reset(X0);


for time = 0:dt:tf
   
    X0 = sim_state.X;
    
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    tau = robot.getTorqueCommand(X0, time);
    
    dynamics=[A,B,C,G];
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G,tau, mu), [0, dt], X0);
    lX = length(X);
    sim_state = sim_state.updateState(X(lX,:), time+dt);

end

sim_state.plotQ(tf);
sim_state.animate(dt, tf, L_1, L_2,1);

























