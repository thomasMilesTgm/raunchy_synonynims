function dXdt = Task1_dynamics(t, X, A, B, C, G, tau, mu)
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