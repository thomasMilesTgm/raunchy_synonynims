close all
clear all
clc

set(0,'defaulttextInterpreter','latex') % LaTex is the only way


%% Setup Simulation Environment

% initial state
X0 = 2*pi/360.*[30, 75, 0, 0];
dt = 0.01;
dt_pid = 0.001;
tf = 5;
mdl_err = 0; % robot modeling error

% Reference velocity trajectory U(t) = 3*at^2 + 2*bt + c
a=[pi/250, pi/375]; b=[-3*pi/100, -pi/50]; c=[0,0]; d=[0,0];

ref = @(t)(3.*a.*t.^2 + 2.*b.*t + c);

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

% Create state and robot for simulation
sim_state = State(X0, A, B, C, G);
robot = Robot(X0,mdl_err);

% make sure tau is initialized (not actually necessary but good practice)
U = ref(0);
tau = [0,0];

% Simulate
for time = 0:dt_pid:tf
   
    X0 = sim_state.X; 
    A = sim_state.getA;
    B = sim_state.getB;
    C = sim_state.getC;
    G = sim_state.getG;
    
    % calculate refernce velocity
    U = ref(time);
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


% sim_state.animate(dt, tf, L_1, L_2,10);
sim_state.plotQ(tf);

sim_state.plotQDot(tf);

% plot reference trajectory
r=[0,0];
i=1;
r2d=360/(2*pi);
for t=0:dt_pid:tf
   r(i,:)=r2d.*ref(t);
   i=i+1;
end

t=0:dt_pid:tf;
plot(t,r(:,1),'LineWidth',2,'LineStyle',':');
plot(t,r(:,2),'LineWidth',1,'LineStyle',':');
legend('\dot{q_1}','\dot{q_2}','\dot{q_{1ref}}','\dot{q_{2ref}}');
axis([0,5,-30,10])

% 

robot.plotError()
grid()

% robot.plotTau(0)
