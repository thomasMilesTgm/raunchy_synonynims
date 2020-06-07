close all
clear all
clc
set(groot,'defaultLegendInterpreter','latex') % LaTex is the only way
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
   
    X0 = sim_state.Q;
    
    A = sim_state.getA();
    B = sim_state.getB();
    C = sim_state.getC();
    G = sim_state.getG();
    
    dynamics=[A,B,C,G];
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G, tau, mu), [0, dt], X0);
    lX = length(X);
    sim_state = sim_state.updateState(X(lX,:), time+dt);

end

%% PLOTS 1.2
% sim_state.animate(dt, tf, L_1, L_2,1);
sim_state.plotQ(tf);
legend('$q_1$','$q_1$', 'FontSize',20);


%% Robot Vs. Gravity



mdl_err = 0.001;     % modelling error (decimal percent of all params)
dt = 0.01;
tf = 5;
mu = 0.1;           % Friction coefficient
X0 = [0, 0, 0, 0];  % initial state

% The robot has its on state object so it's easy to add error in it's model
% parameters to simulate real world errors neatly.
[A,B,C,G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy);

robot = Robot(X0,mdl_err,1,[-pi,pi]);
sim_state = State(X0, A, B, C, G);
% tau = robot.getTorqueCommand(X0, time);

for time = 0:dt:tf
   
    X0 = sim_state.Q;
    
    A = sim_state.getA();
    B = sim_state.getB();
    C = sim_state.getC();
    G = sim_state.getG();
    
    [robot,tau] = robot.getTorqueCommand(X0, time);
    
    dynamics=[A,B,C,G];
    
    [t,X] = ode45(@(t,y) Task1_dynamics(t,X0, A, B, C, G,tau, mu), [0, dt], X0);
    lX = length(X);
    sim_state = sim_state.updateState(X(lX,:), time+dt);

end
%% PLOTS 1.3
figure()
sim_state.plotQ(tf);
sim_state.plotQDot(tf);
legend('$q_1$','$q_1$','$\dot{q}_1$','$\dot{q}_1$', 'FontSize',20);
figure()
hold on
line([0,5],[robot.tau_last(1),robot.tau_last(1)],'Color','b','LineWidth',2);
line([0,5],[robot.tau_last(2),robot.tau_last(2)],'Color','r','LineWidth',2);
legend('$\tau_1$','$\tau_2$', 'FontSize',20)

xlabel('time (sec)', 'FontSize',20)
ylabel('Torque (Nm)', 'FontSize',20)
% sim_state.animate(dt, tf, L_1, L_2,1);


























