close all;
clear all;

dt = 0.1; 
tf = 5; 

% Initialise the robot to the initial position and velocity
q(1)= 0/180*pi; % Converting degree to radian
q(2) = 0.6; 
qdot(1) = 0;
qdot(2) = 0;
i = 1;

for time=0:dt:tf

    tau = [0;0];  % This is the command torque, for now it is set to zero
    [t,y] = ode45(@(t,y) runrobot(t,y,tau), [0, dt], [q(1), q(2), qdot(1), qdot(2)])
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    if q(2) < 0.5
        q(2) = 0.5;
        qdot(2) = 0;
    elseif q(2) > 1
        q(2) = 1;
        qdot(2) = 0;
    end

    %%% Storing q and qdot values into an array
    q1s(i) = q(1);
    q2s(i) = q(2);
    qdot1s(i) = qdot(1);
    qdot2s(i) = qdot(2);
    i = i+1;

end

time = 0:dt:tf;
plot(time,q1s,'-o', time,q2s,'-o')
legend('q1','q2')
