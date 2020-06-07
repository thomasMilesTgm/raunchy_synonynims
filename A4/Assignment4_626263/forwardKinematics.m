function [x,y] = forwardKinematics(q1, q2, l1, l2)

    x1=cos(q1)*l1;
    y1=sin(q1)*l1;
    r1_2 = [cos(q2)*l2; sin(q2)*l2];
    R1_0 = [cos(q1), -sin(q1); sin(q1), cos(q1)];
    
    r0_2 = R1_0*r1_2 + [x1;y1];
    
    x = r0_2(1);
    y = r0_2(2);
%     dx = -(q1+q2)*sin(q1+q2)*l2 - l1*q1*sin(q1);
%     dy = (q1+q2)*cos(q1+q2)*l2 + l1*q1*cos(q1);
%     
%     X = [x,dx];
%     Y = [];
end