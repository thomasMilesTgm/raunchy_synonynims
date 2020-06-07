close all
clear all
clc

set(groot,'defaulttextInterpreter','latex') % LaTex is the only way
set(groot,'defaultLegendInterpreter','latex') % LaTex is the only way

anim=0; % do you want to animate
%% Setup Simulation Environment

% initial state
Q0 = 2*pi/360.*[30, 75, 0, 0];
dt = 0.01;
dt_pid = 0.001;
tf = 5;
mdl_err = 0.05; % robot modeling error
pos_sensor = 1; % does the robot hava a position sensor?

% Reference velocity trajectory U(t) = 3*at^2 + 2*bt + c
a=[pi/250, pi/375]; b=[-3*pi/100, -pi/50]; c=[Q0(3),Q0(4)]; d=[Q0(1),Q0(2)];

ref = @(t)(3.*a.*t.^2 + 2.*b.*t + c);
q_ref = @(t)(a.*t.^3 + b.*t.^2 + c.*t + d);

% True Physical Parameters
m_1 = 2; %kg
m_2 = 1; %kg
L_1 = 1; %m
L_2 = 0.6; %m
r_c1 = L_1/2; %m
r_c2 = L_2/2; %m

I_zz1 = 0.5;    %kgm^2
I_zz2 = 0.3;    %kgm^2
I_xx = 0;       %kgm^2
I_yy = 0;       %kgm^2

mu = 0.05; % Friction coefficient

% Generate dynamics
[A,B,C,G] = formulteDynamics(...
    m_1, m_2, L_1, L_2, r_c1, r_c2, I_zz1, I_zz2, I_xx, I_yy);

% Create state and robot for simulation
sim_state = State(Q0, A, B, C, G);
robot = Robot(Q0,mdl_err, pos_sensor, [-pi,pi]);

% make sure tau is initialized (not actually necessary but good practice)
U = ref(0);
tau = [0,0];

%% Simulation loop
for time = 0:dt_pid:tf
   
    Q_now = sim_state.Q; 
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    % calculate refernce velocity
    U = ref(time);
    

    % Give the robot current velocity "measurement"
    robot = robot.sample(Q_now, U, time);
    

    % Get new command from robot every dt
    if mod(time, dt) == 0
        [robot,tau] = robot.PID(U, time);  
    end

    
    [~,X] = ode45(@(t,y) Task1_dynamics(t,Q_now, A, B, C, G, tau, mu), [0, dt_pid], Q_now);
    lX = length(X);
    sim_state = sim_state.updateState(X(lX,:), time+dt_pid);

end

%% Plots & Animations

% sim_state.animate(dt, tf, L_1, L_2,10);

r2d=360/(2*pi); % rad 2 deg
time=0:dt_pid:tf;

% plot reference and actual joint angles
figure()
hold on

sim_state.plotQ(tf); % actual

i=1;
q_r=[0,0];
for t=time
    q_r(i,:) = r2d.*q_ref(t);
    i=i+1;
end

plot(time, q_r,'LineStyle','--', 'LineWidth', 2);

    
legend('$q_1$', '$q_2$', '$q_{1r}$', '$q_{2r}$', 'FontSize',20)

figure()
hold  on

% plot reference trajectory and actual velocity
sim_state.plotQDot(tf);
r=[0,0];
i=1;

for t=0:dt_pid:tf
   r(i,:)=r2d.*ref(t);
   i=i+1;
end


plot(time,r(:,1),'LineWidth',2,'LineStyle','--');
plot(time,r(:,2),'LineWidth',2,'LineStyle','--');
robot.plotError()
legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_{1ref}$','$\dot{q}_{2ref}$', '$e_1$', '$e_2$', 'FontSize',20);
axis([0,5,-15,2])

% 
figure()
hold on


robot.plotTau()
legend('$\tau_1$','$\tau_2$', 'FontSize',20)

%% PLOT 2.3 
if anim
    close all
    sim_state.animate(dt, 0.5, L_1, L_2,10*50);
    grid
    axis([0,1.6,-0.4,1.2])
    daspect([1 1 1])
    xlabel('x', 'FontSize',20)
    ylabel('y', 'FontSize',20)
    sim_state.plotEE_path(L_1,L_2)
end
