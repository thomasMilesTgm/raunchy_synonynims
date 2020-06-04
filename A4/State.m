classdef State
   properties 
    
       X            % Current State Vector [q1, q2, q1dot, q2dot]
       X_history    % Past States [time, q1, q2, q1dot, q2dot]
       A            % Inertia matrix
       B            % Coriolis matrix
       C            % Centrifugal matrix
       G            % Gravity matrix
       h_len=1      % Number of states in history    
   end
   
   methods
      
       function obj = State(X0, A, B, C, G)
           obj.X = X0;
           obj.X_history = [0, X0];
           % syms is terrible. Converting to matlab functions makes this
           % run literally hundreds of times faster.          
           obj.A = matlabFunction(A);
           obj.B = matlabFunction(B);
           obj.C = matlabFunction(C);
           obj.G = matlabFunction(G);
           
       end
       
       function obj = reset(obj, X0)
           % Resets history and initial state
           obj.X = X0;
           obj.X_history = [0, X0];
           obj.h_len=1;
           
       end
       
       function obj = updateState(obj, X, t)
            obj.h_len = obj.h_len+1;
            obj.X_history(obj.h_len,:) = [t,X];
            obj.X = X;
       end

       
       function A = getA(obj)
           
           q_2 = obj.X(2);
           A = obj.A(q_2);
       end
       
       function B = getB(obj)
           
           q_2 = obj.X(2);
           B = obj.B(q_2);
       end
       
       function C = getC(obj)
           
           q_2 = obj.X(2);
           C = obj.C(q_2);
       end      
       
       function G = getG(obj)
           q_1 = obj.X(1);
           q_2 = obj.X(2);
           G = obj.G(q_1,q_2);
       end     
       
       
       function animate(obj,dt,tf,L_1,L_2,skip)
           hold off
           
            i=1;
               
            
            anim=figure(1);

            for t=0:dt:tf

                clf(anim);
                
                axis([-2,2,-2,2]);
                
                
                
                q1 = obj.X_history(i,2);
                q2 = obj.X_history(i,3);
                plotArm(L_1,L_2,q1,q2);
                
                % Timer
                text(1.5,2, 'time: ' , ...
                      'HorizontalAlignment', 'right', ...
                      'VerticalAlignment', 'top');
                  
                t_str = string(obj.X_history(i,1));
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
            end
            
       end
       
       
       function plotQ(obj,tf)
           
           r2d=360/(2*pi);
           
           figure();
           hold on;
           plot(obj.X_history(:,1), r2d.*obj.X_history(:,2),'LineWidth',3);
           plot(obj.X_history(:,1), r2d.*obj.X_history(:,3),'LineWidth',3);
           legend('q_1','q_2');
           xlabel('time (s)');
           ylabel('joint angle (deg)');
           grid
           xlim([0,tf]);
       end
       
      function plotQDot(obj,tf)
          r2d=360/(2*pi);
          
           figure();
           hold on;
           plot(obj.X_history(:,1), r2d.*obj.X_history(:,4),'LineWidth',1);
           plot(obj.X_history(:,1), r2d.*obj.X_history(:,5),'LineWidth',1);
           legend('\dot{q_1}','\dot{q_2}');
           xlabel('time (s)');
           ylabel('joint velocity (deg/s)');
           grid
           xlim([0,tf]);
       end
   end
end