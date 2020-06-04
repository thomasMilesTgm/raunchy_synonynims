function [x,y] = forwardKinematics(q1, q2, l1, l2)

    x1=cos(q1)*l1;
    y1=sin(q1)*l1;
    
    x = cos(q1+q2)*l2+x1;
    y = sin(q1+q2)*l2+y1;
    
    

end