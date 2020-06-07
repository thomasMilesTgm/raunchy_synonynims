close all
clear all
clc
set(groot,'defaulttextInterpreter','latex') % LaTex is the only way
set(groot,'defaultLegendInterpreter','latex') % LaTex is the only way

Trajectory='B';
anim=0;
% proportional gains for task space controller
P1 = 14;
P2 = 10;
P = [P1,0; 0,P2];

%% Setup Simulation Environment
syms t
up = 0; % manual elbow position
has_lim=0; 

show_tr=0;

% initial state
dt = 0.01;
dt_pid = 0.001;
tf = 5;
mdl_err = 0; % robot modeling error
pos_sensor =1;

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


% error and U histories
e_hist = [0,0];
u_hist = [0,0];



%% Trajectory A
if Trajectory=='A'
    X0 = [0.710; 0]; Y0 = [1.080; 0]; % initial state
    Xf = [1.485; 0]; Yf = [0.041; 0]; % final state
elseif Trajectory=='B'
    X0 = [0.710; 0]; Y0 = [1.080; 0]; % initial state
    Xf = [1.36; 0]; Yf = [-0.59; 0]; % final state
end


[Xref, Yref] = trajectoryGen(X0, Y0, Xf, Yf, tf, show_tr);


% Generate joint space trajectories for the robot and simulator
[Q1_ref_fn, Q2_ref_fn] = inverseKinematics(Xref(t), Yref(t), L_1, L_2, up, 1);

% initial joint space conditions
Q1_0=Q1_ref_fn(0);
Q2_0=Q2_ref_fn(0);

Q0=[Q1_0(1), Q2_0(1), Q1_0(2), Q2_0(2)];

% Create state and robot for simulation
sim_state = State(Q0, A, B, C, G);
robot = Robot(Q0, mdl_err, pos_sensor, [Q1_max, Q2_max]);

U =[0,0];
x_last = X0(1);
y_last = Y0(1);
i=1;
%% Simulation loop
for time = 0:dt_pid:tf
    
    is_lim=0;
    Q_now = sim_state.Q; 
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    % calculate error in task space to generate reference velocity signal U
    % for the robot
    e_x = Xref(time) - x_last;
    e_y = Yref(time) - y_last;
    e = [e_x(1); e_y(1)];
    
    %
    u_hist(i,:) = U;
    e_hist(i,:) = e';

    
    
    
    
    % Proportional task space controller using position error
    q_1 = Q_now(1);
    q_2 = Q_now(2);
    
    J_v2 = [-L_1*sin(q_1) - r_c2*sin(q_1 + q_2), -r_c2*sin(q_1 + q_2);...
            L_1*cos(q_1) + r_c2*cos(q_1 + q_2), r_c2*cos(q_1 + q_2)];
    
    
     
    % reference velocity for robot
    
    robot = robot.sample(Q_now, U, time);
    
    % Get new command from robot every dt
    if mod(time, dt) == 0
        U = (inv(J_v2)*P*e)';
        [robot,tau] = robot.PID(U, time);  
        [x_last,y_last] = forwardKinematics(q_1,q_2,L_1,L_2);
        
        
        
    end

    
    [~,X] = ode45(@(t,y) Task3_dynamics(t,Q_now, A, B, C, G, tau, mu, Q1_max, Q2_max), [0, dt_pid], Q_now);
    lX = length(X);
    
    % check joint limits
    Qnext = X(lX,:);
    

%     Joint 1
    if Qnext(1) < Q1_min && Qnext(3)<0
        has_lim=1;
        Qnext(1) = Q1_min;
        Qnext(3) = 0;
    elseif Qnext(1) > Q1_max && Qnext(3)>0
        has_lim=1;
        Qnext(1) = Q1_max;
        Qnext(3) = 0;
    end
%      Joint 2
    if Qnext(2) < Q2_min && Qnext(4)<0
        has_lim=1;
        Qnext(2) = Q2_min;
        Qnext(4) = 0;
    elseif Qnext(2) > Q2_max&& Qnext(4)>0
        has_lim=1;
        Qnext(2) = Q2_max;
        Qnext(4) = 0;
    end

        
    % update state
    sim_state = sim_state.updateState(Qnext, time+dt_pid);
    i=i+1;
    
end


%% Plots & Animations

% sim_state.animate(dt, tf, L_1, L_2,10);

sim_state.plotEE_path(L_1, L_2)


r2d=360/(2*pi); % rad 2 deg
time=0:dt_pid:tf;



% plot reference joint velocity input vs and actual velocity
figure()
hold on

sim_state.plotQDot(tf);
plot(time,u_hist(:,1));
plot(time,u_hist(:,2));

robot.plotTau()

legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_{1ref}$','$\dot{q}_{2ref}$','$\tau_1$','$\tau_2$');


axis([0,5,-35,30])

% 

figure()
hold on
plot(time,e_hist(:,1));
plot(time,e_hist(:,2));
robot.plotTau()

legend('$e_x$','$e_y$','$\tau_1$','$\tau_2$');

% plot end effector x and y vs t
time=0:dt_pid:tf;
xr=Xref(time);
yr=Yref(time);


figure()
hold on


sim_state.plotEE_t(L_1,L_2, 'x')
sim_state.plotEE_t(L_1,L_2, 'y')
plot(time,yr(1,:),'--','LineWidth',2)
plot(time,xr(1,:),'--','LineWidth',2)

legend('$x_e$','$y_e$','$x_r$','$y_r$', 'FontSize',20)
grid()
xlabel('time (sec)', 'FontSize',20)
ylabel('position (m)', 'FontSize',20)

figure()
resolution = pi/30;
robot.plotReachable(resolution)
sim_state.plotEE_path(L_1, L_2)



if anim
    close all
    sim_state.animate(dt, 0.5, L_1, L_2,10*50);
    grid
    
    daspect([1 1 1])
    xlabel('x', 'FontSize',20)
    ylabel('y', 'FontSize',20)
    
    
    if Trajectory=='A'
        axis([0,1.6,-0.4,1.2])
    else
        axis([0,1.4,-0.8,1.2])
    end
end














