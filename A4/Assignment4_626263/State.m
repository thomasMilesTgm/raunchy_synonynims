classdef State
   properties 
    
       Q            % Current State Vector [q1, q2, q1dot, q2dot]
       Q_history    % Past States [time, q1, q2, q1dot, q2dot]
       A            % Inertia matrix
       B            % Coriolis matrix
       C            % Centrifugal matrix
       G            % Gravity matrix
       h_len=1      % Number of states in history    
   end
   
   methods
      
       function obj = State(Q0, A, B, C, G)
           obj.Q = Q0;
           obj.Q_history = [0, Q0];
           % syms is terrible. Converting to matlab functions makes this
           % run literally hundreds of times faster.          
           obj.A = matlabFunction(A);
           obj.B = matlabFunction(B);
           obj.C = matlabFunction(C);
           obj.G = matlabFunction(G);
           
       end
       
       function obj = reset(obj, Q0)
           % Resets history and initial state
           obj.Q = Q0;
           obj.Q_history = [0, Q0];
           obj.h_len=1;
           
       end
       
       function obj = updateState(obj, Q, t)
            obj.h_len = obj.h_len+1;
            obj.Q_history(obj.h_len,:) = [t,Q];
            obj.Q = Q;
       end

       
       function A = getA(obj)
           
           q_2 = obj.Q(2);
           A = obj.A(q_2);
       end
       
       function B = getB(obj)
           
           q_2 = obj.Q(2);
           B = obj.B(q_2);
       end
       
       function C = getC(obj)
           
           q_2 = obj.Q(2);
           C = obj.C(q_2);
       end      
       
       function G = getG(obj)
           q_1 = obj.Q(1);
           q_2 = obj.Q(2);
           G = obj.G(q_1,q_2);
       end     
       
       
       function animate(obj,dt,tf,L_1,L_2,skip)
           
           if skip<20
            hold off
           else
              hold on 
           end
            i=1;
               
            
            anim=figure(1);

            for t=0:dt:tf
                if skip<20
                    clf(anim);
                
                axis([-2,2,-2,2]);
                end
                
                
                q1 = obj.Q_history(i,2);
                q2 = obj.Q_history(i,3);
                plotArm(L_1,L_2,q1,q2);
                
                % Print Timer
                text(1.5,2, 'time: ' , ...
                      'HorizontalAlignment', 'right', ...
                      'VerticalAlignment', 'top');
                  
                t_str = string(obj.Q_history(i,1));
                L = strlength(t_str);
                while L < 6
                   t_str = join([t_str, ' ']);
                    L = L+1;
                end
                
                text(2,2, t_str , ...
                      'HorizontalAlignment', 'right', ...
                      'VerticalAlignment', 'top');
                pause(dt);
                i=i+skip;
                if i > obj.h_len
                    break
                end
            end
            
       end
       
       
       
       function plotEE_path(obj, L1, L2)
           t  = obj.Q_history(:,1);
           q1 = obj.Q_history(:,2);
           q2 = obj.Q_history(:,3);
           
           x1 = L1.*cos(q1);
           y1 = L1.*sin(q1);
           
           xe = x1 + L2*cos(q1 + q2);
           ye = y1 + L2*sin(q1 + q2);
           
           plot(xe,ye,'LineWidth',3,'Color','r');
       end
       
       function plotEE_t(obj, L1, L2, x)
           
           t  = obj.Q_history(:,1);
           q1 = obj.Q_history(:,2);
           q2 = obj.Q_history(:,3);
           
           x1 = L1.*cos(q1);
           y1 = L1.*sin(q1);
           
           xe = x1 + L2*cos(q1 + q2);
           ye = y1 + L2*sin(q1 + q2);
           if x=='x'
               plot(t,xe,'-','LineWidth',2,'Color','r');
           else
               plot(t,ye,'-','LineWidth',2,'Color','b');
           end
       end
       
       
       function plotQ(obj,tf)
           hold on
           r2d=360/(2*pi); % rad to deg
           plot(obj.Q_history(:,1), r2d.*obj.Q_history(:,2),'LineWidth',2);
           plot(obj.Q_history(:,1), r2d.*obj.Q_history(:,3),'LineWidth',2);
           
           xlabel('time (s)', 'FontSize',20);
           ylabel('joint angle (deg)', 'FontSize',20);
           grid
           xlim([0,tf]);
       end
       
      function plotQDot(obj,tf)
          r2d=360/(2*pi);
          
           
           plot(obj.Q_history(:,1), r2d.*obj.Q_history(:,4),'LineWidth',2);
           plot(obj.Q_history(:,1), r2d.*obj.Q_history(:,5),'LineWidth',2);
%            legend('\dot{q_1}','\dot{q_2}');
           xlabel('time (s)', 'FontSize',20);
           ylabel('joint velocity (deg/s)', 'FontSize',20);
           grid
           xlim([0,tf]);
       end
   end
end