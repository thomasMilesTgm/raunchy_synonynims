function dydt = runrobot(t,y,tau)
    
    % hard limits on joint2
    if y(2) < 0.5
        y(2) = 0.5;
        y(4) = 0;
    elseif y(2) > 1
        y(2) = 1;
        y(4) = 0;
    end
    % x represents the state that contains q and qdot for the robot 
    % ie. for both joints
    q = [y(1); y(2)];
    qdot = [y(3); y(4)];
    q1 = y(1);
    d2 = y(2); % note this is a prismatic joint. d2 minimum is rc1
    q1dot = y(3);
    d2dot = y(4);

    m1 = 2; %(kg)
    m2 = 1; %(kg)
    Izz1 = 1; 
    Izz2 = 0.6;
    rc1 = 0.4; % (m) distance to center of mass 1
    rc2 = d2; %(distance to center of mass 2 from origin -- note it is a prismatic joint, see Fig)
    g = 9.8;

    % inertia matrix
    A = [m1*rc1^2+ Izz1 + m2*d2^2 + Izz2, 0; 0, m2];
    % Coriolis and  centrifugal
    B = [2*m2*d2; 0];
    C = [0, 0; -m2*d2, 0];

    % gravity term
    G = [(m1*rc1 + m2*d2) *g *cos(q1); m2*g*sin(q1)];

    % friction
    friction_coeff = 0;
    friction = friction_coeff * [q1dot ; d2dot];
    
    %Calculating the qdoubledot -- joint space acceleration
    qddot = inv(A) * (tau - friction - B * (q1dot *d2dot) - C * [q1dot^2; d2dot^2] - G); % this is qdoubledot.

    dydt = zeros(4,1);
    dydt(1) = y(3);
    dydt(2) = y(4);
    dydt(3) = qddot(1);
    dydt(4) = qddot(2);
    
end