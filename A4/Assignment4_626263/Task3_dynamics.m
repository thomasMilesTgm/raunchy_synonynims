
function dXdt = Task3_dynamics(t, X, A, B, C, G, tau, mu, q1_lim, q2_lim)
% does the same as task1_dynamics, but enforces joint limits
    
    if X(1) > q1_lim
         X(1) = q1_lim;
         X(3) = 0;
    elseif  X(1) < -q1_lim
         X(1) = -q1_lim;
         X(3) = 0;
    end 
    if X(2) > q2_lim
         X(2) = q2_lim;
         X(4) = 0;
    elseif  X(2) < -q2_lim
         X(2) = -q2_lim;
         X(4) = 0;
    end     
         
    % friction
    friction = mu * [X(3) ; X(4)];
    
    % joint space accelerations
    dQ2dt2 = inv(A) * (tau' - friction - B * (X(3) *X(3)) ...
                        - C * [X(3)^2; X(4)^2] - G);
       
   
         
    dXdt = zeros(4,1);
    dXdt(1) = X(3);
    dXdt(2) = X(4);
    dXdt(3) = dQ2dt2(1);
    dXdt(4) = dQ2dt2(2);


end
