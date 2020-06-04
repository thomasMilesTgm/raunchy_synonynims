function plotArm(l1,l2,q1,q2)
    
    
    
    
    l1x = l1*cos(q1);
    l1y = l1*sin(q1);
    
    l2x = l1x + l2*cos(q2);
    l2y = l1y + l2*sin(q2);
    
    line([0,l1x],[0,l1y],'LineWidth',10);
    line([l1x,l2x], [l1y,l2y],'LineWidth',10);
  
end