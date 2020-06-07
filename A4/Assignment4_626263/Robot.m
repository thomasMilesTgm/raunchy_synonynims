classdef Robot
   
    properties 
        
        has_pos_sensor;
        G_estimator=1;  % turns on and off the gravity estimation for controller
        
        % State object so robot can simulate it's on dynamics
        state

        % Physical parameters for dynamics model
        m_1=2; %kg
        m_2=1; %kg
        L_1=1; %m
        L_2=0.6; %m


        I_zz1=0.5; %kgm^2
        I_zz2=0.3; %kgm^2
        I_xx=0; %kgm^2
        I_yy=0; %kgm^2
        
        
        r_c1;
        r_c2;
        I_1;
        I_2;
        
        % PID and task2 things
        U_hist=zeros(10,2); % Reference history buffer
        V_hist=zeros(10,2); % Velocity history buffer
        V_last=zeros(1,2);  % last velocity measurement
        e_last=zeros(1,2);  % last error
        samples=0;          % samples in history buffers
        dt_pid=0.001;       % pid sampling rate
    
        pid;
        
        
        K_p1 = 100;    % proportional gain - joint 1
        K_i1 = 200;    % integral gain - joint 1
        K_d1 = 0.5;    % differential gain - joint 1
        
        
        K_p2 = 20;    % proportional gain - joint 2
        K_i2 = 50;    % integral gain - joint 2
        K_d2 = 0.05;  % differential gain - joi        
        
        
        
        K_p;
        K_i;
        K_d;
        
        K = 1;   % feedback gain
        
        % Control history
        h_len=1;
        e_hist=zeros(500,2);
        tau_hist=zeros(500,2);
        t_hist=zeros(500,1);
        tau_last=[0,0];
        
        % Joint limits [q1, q2]
        Q_lim;
        
    end
    
    methods
        function obj = Robot(X0, mdl_err, pos_sensor, limits)
            % model error
            e = 1+mdl_err;
            
            obj.has_pos_sensor = pos_sensor;
            obj.Q_lim = limits;
            
            % Initialize dependant params
            obj.r_c1 = e*obj.L_1/2; %m
            obj.r_c2 = e*obj.L_2/2; %m

            obj.I_1 = e.*[ obj.I_xx, 0, 0;
                    0, obj.I_yy, 0;
                    0, 0, obj.I_zz1];

            obj.I_2 = e.*[ obj.I_xx, 0, 0;
                    0, obj.I_yy, 0;
                    0, 0, obj.I_zz2];
            
            % Incorpeorate modelling errors
            obj.m_1=e*obj.m_1;
            obj.m_2=e*obj.m_2;
            obj.L_1=e*obj.L_1;
            obj.L_2=e*obj.L_2;
            obj.I_zz1=e*obj.I_zz1;
            obj.I_zz2= e*obj.I_zz2;
            obj.I_xx=e*obj.I_xx;
            obj.I_yy=e*obj.I_yy;
            
            
            % Initialize PID Matricies and controller
            obj.K_p = [obj.K_p1, obj.K_p2];
            obj.K_i = [obj.K_i1, obj.K_i2];
            obj.K_d = [obj.K_d1, obj.K_d2];
            
            obj.pid= pid(obj.K_p, obj.K_i,obj.K_d,...
                            'Ts', 0.001, 'Tf', 0.01, 'IFormula', 'Trapezoidal');
            
                
            % Generate dynamics for robot's state object
            [A,B,C,G] = formulteDynamics(...
                obj.m_1, obj.m_2, obj.L_1, obj.L_2, obj.r_c1, obj.r_c2, ...
                obj.I_zz1, obj.I_zz2, obj.I_xx, obj.I_yy);
            
            obj.state = State(X0, A, B, C, G);
            
        end
        
        
        function [obj,tau] = getTorqueCommand(obj, measurement, t)
            % given a measured state, the robot will return a tourque
            % command in an attemt to defy gravity by applying an equal
            % counter torque
            
            obj.state=obj.state.updateState(measurement, t);
            
            tau = obj.state.getG()';
            
            
            obj.tau_last = tau;
            
        end
        
        
        function obj=sample(obj, Q, U, t)
           % Record a velocity measurement V and the current
           % velocity reference U and update the robot's state object
           
           
           % if the robot has a position sensor it can just save the true 
           % position, otherwise it will estimate it from velocity
           if obj.has_pos_sensor
                V = [Q(3), Q(4)];
                obj.state = obj.state.updateState(Q,t);
                
           else
               % integrate to estimate q1,q2
                Q_last = obj.state.Q;
                A = obj.state.getA;
                B = obj.state.getB;
                C = obj.state.getC;
                G = obj.state.getG;
                

                [~,X] = ode45(@(t,y) Task1_dynamics(t,Q_last, A, B, C, G, obj.tau_last, 0.01), [0, 0.001], Q_last);
                lX = length(X);
                
                Q_est = X(lX,:);
                Q_combined = [Q_est(1), Q_est(2), Q(3), Q(4)]; % estimated pos, measured v
                
                obj.state = obj.state.updateState(Q_combined, t + 0.001);
                Q_est = obj.state.Q;
                
                V = [Q_est(3), Q_est(4)];
           end
         
            obj.V_last = V;
            obj.e_last = U-V;
            
        end
        
        
        function [obj,tau] = PID(obj, U, t) 
            % error
            e = U - obj.V_last;
            
            % get PID output
            tau_1 = lsim(obj.pid(:,:,1,1), [obj.e_last(1), e(1)], [0,obj.dt_pid]);
            tau_2 = lsim(obj.pid(:,:,1,2), [obj.e_last(2), e(2)], [0,obj.dt_pid]);
            
            tau = [tau_1(2), tau_2(2)];
            
            if obj.G_estimator
            % incorporate G into torque commnad
                tau = tau + obj.state.getG()';
            end
            % Store error and torque for plotting
            obj.e_hist(obj.h_len,:)=obj.e_last ;
            
            obj.e_hist(obj.h_len, :)=e';
            obj.tau_hist(obj.h_len, :)=tau;
            obj.t_hist(obj.h_len)=t;
            obj.h_len=obj.h_len+1;
            obj.tau_last = tau;
            
            
        end

        
        function plotError(obj)
            r2d=360/(2*pi);
           
            hold on
            plot(obj.t_hist, r2d.*obj.e_hist(:,1),':','LineWidth',2);
            plot(obj.t_hist, r2d.*obj.e_hist(:,2),':','LineWidth',2);

        end
        
        
        function plotTau(obj)
            
            plot(obj.t_hist, obj.tau_hist(:,1),':','color','k','LineWidth',2);
            plot(obj.t_hist, obj.tau_hist(:,2),':','color','r','LineWidth',2);
            ylabel('Torque (Nm)','FontSize',20)
            xlabel('time (s)','FontSize',20)
%             legend();
        end
        
        function plotReachable(obj, resolution)
            figure()
            axis([-0.2,1.6,-1.5,1.5]);
%             daspect([1 1 1])
            
            hold on
            q1 = -obj.Q_lim(1);
            q2 = -obj.Q_lim(2);
%             plot(0,0,'Marker','o','Color','k','MarkerSize',5)
            while q1 <= obj.Q_lim(1)
                x1=cos(q1)*obj.L_1;
                y1=sin(q1)*obj.L_1;
%                 plot(x1,y1,'Marker','o', 'Color','k','MarkerSize',5);
                 while q2 <= obj.Q_lim(2)
                                      
                   [x,y] = forwardKinematics(q1, q2, obj.L_1, obj.L_2);
                   
                   c = abs(q2-obj.Q_lim(2))/(2*obj.Q_lim(2));
                   
                   plot(x,y,'h','Color',[0,0,0.5],'MarkerSize',4);
                   
                   
                   q2 = q2+resolution*0.5;
%                    pause(0.00001);
                 end
                 
                q1 = q1 + resolution;
                q2 = -obj.Q_lim(2);
            end
        end
        
        
    end
end