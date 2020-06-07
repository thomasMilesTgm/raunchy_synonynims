function [Q1, Q2] = inverseKinematics(X, Y, l1, l2, up, fn)
% returns elbow up kinematics if up is true

    t=sym('t');
    
    D = acos((X(1).^2 + Y(1).^2 - l1^2 - l2^2)/(2*l1*l2));
    
    if up
        Q2 = -D;
    else
        Q2 = D;
    end

    gamma = abs(pi-Q2);
    alpha = asin((l2*sin(gamma))/sqrt(X(1).^2 + Y(1).^2));
    beta = atan(Y(1)./X(1));

    Q1 = beta-alpha;

    
    if fn
        Q1 = vpa([Q1; diff(Q1,t)]);
        Q2 = vpa([Q2; diff(Q2,t)]);
    %     
        Q1 = matlabFunction(Q1);
        Q2 = matlabFunction(Q2);
    end
end