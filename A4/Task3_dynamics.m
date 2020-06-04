%% Converts reference trajectory to joint space through inverse kinematics

function [Q1, Q2] = Task3_dynamics(X, Y, l1, l2, up)

    Q1_min = -60*2*pi/360;
    Q1_max = 60*2*pi/360;
    Q2_min = -90*2*pi/360;
    Q2_max = 90*2*pi/360;

    t=sym('t');
    % returns elbow up kinematics if up is true
    D = acos((X(1).^2 + Y(1).^2 - l1^2 - l2^2)/(2*l1*l2));
    
%     if -D < Q2_min
%        up=0; 
%     end
%     
%     if D > Q2_max
%        up=1; 
%     end
    
    if up
        Q2 = -D;
    else
        Q2 = D;
    end

    gamma = abs(pi-Q2);
    alpha = asin((l2*sin(gamma))/sqrt(X(1).^2 + Y(1).^2));
    beta = atan(Y(1)./X(1));

    Q1 = beta-alpha;

    
    
    Q1 = vpa([Q1; diff(Q1,t)]);
    Q2 = vpa([Q2; diff(Q2,t)]);
    
    Q1 = matlabFunction(Q1);
    Q2 = matlabFunction(Q2);
 
    % plot
%     figure()
%     i=1;
%         t=0:0.1:5;
%         q1=zeros(length(t),2);
%         q2=zeros(length(t),2);
%         for t=0:0.1:5
%             q1(i,:)=Q1(t);
%             q2(i,:)=Q2(t);
%             i=i+1;
%         end
%         figure()
%         hold on
%         plot(0:0.1:5,q1*360/(2*pi))
%         plot(0:0.1:5,q2*360/(2*pi))
% 
%         legend('Q_1','dQ_1','Q_2','dQ_2');

end