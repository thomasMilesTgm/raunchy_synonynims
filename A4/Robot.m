classdef Robot
   
    properties 
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
        
        
        K_p1 = 1;    % proportional gain - joint 1
        K_i1 = 400;    % integral gain - joint 1
        K_d1 = 0.6;    % differential gain - joint 1
        
        %Tu=0.02sec
        
        % Kcr = 45
        % Pcr = 1.8
        K_p2 = 1;    % proportional gain - joint 2
        K_i2 = 1000;    % integral gain - joint 2
        K_d2 = 0.011;  % differential gain - joi        
        
        
        
%         Kcr = 300
        % Pcr = 1.5
%         K_p1 = 60;    % proportional gain - joint 1
%         K_i1 = 500*1.5;    % integral gain - joint 1
%         K_d1 = 0.0153*1.5;    % differential gain - joint 1
%         
%         %Tu=0.02sec
%         
%         % Kcr = 45
%         % Pcr = 1.8
%         K_p2 = 50*0.7;    % proportional gain - joint 2
%         K_i2 = 500.5*1.8;    % integral gain - joint 2
%         K_d2 = 0.022*1.5;  % differential gain - joint 2
        
% -------------------CUNTED IN FUCKERY--------------------------
%         % Kcr = 300
%         % Pcr = 1.5
%         K_p1 = 0.6*400;    % proportional gain - joint 1
%         K_i1 = 0.5*1.5;    % integral gain - joint 1
%         K_d1 = 0.125*1.5;    % differential gain - joint 1
%         
%         %Tu=0.02sec
%         
%         % Kcr = 45
%         % Pcr = 1.8
%         K_p2 = 45*0.7;    % proportional gain - joint 2
%         K_i2 = 1.8*1.5;    % integral gain - joint 2
%         K_d2 = 0.12*1.5;  % differential gain - joint 2
% -------------------CUNTED IN FUCKERY--------------------------
        K_p;
        K_i;
        K_d;
        
        K = 1;   % feedback gain
        
        % Control history
        h_len=1;
        e_hist=zeros(500,2);
        tau_hist=zeros(500,2);
        t_hist=zeros(500,1);
    end
    
    methods
        function obj = Robot(X0, mdl_err)
            % error
            e = 1+mdl_err;
            
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
%             obj.pid = feedback(pidOL, obj.K);
            
                
            % Generate dynamics for state object
            [A,B,C,G] = formulteDynamics(...
                obj.m_1, obj.m_2, obj.L_1, obj.L_2, obj.r_c1, obj.r_c2, ...
                obj.I_zz1, obj.I_zz2, obj.I_xx, obj.I_yy);
            
            obj.state = State(X0, A, B, C, G);
            
        end
        
        
        function tau = getTorqueCommand(obj, measurement, t)
            % given a measured state, the robot will return a tourque
            % command in an attemt to defy gravity by applying an equal
            % counter torque
            
            obj.state.updateState(measurement, t);
            tau = obj.state.getG;
            
        end
        
        
        function obj=sample(obj, V, U)
           % Record a velocity measurement V and the current
           % velocity reference U

           i = obj.samples;
            if i<21
                i = i+1;
                obj.samples = i;                
            else
                obj.samples = 1;
                obj.U_hist=zeros(20,2);
                obj.V_hist=zeros(20,2);
                i=1;
            end
            obj.U_hist(i,:) = U;
            obj.V_hist(i,:) = V;
            
            obj.V_last = V;
            obj.e_last = U-V;
            
        end
        
        
        function [obj,tau] = PID(obj, U, t, dt) 
            
            e = U - obj.V_last;
            
            
            t1 = lsim(obj.pid(:,:,1,1), [obj.e_last(1), e(1)], [0,0.001]);
            t2 = lsim(obj.pid(:,:,1,2), [obj.e_last(2), e(2)], [0,0.001]);
            
            tau = [t1(2), t2(2)];
% Store error and torque
            obj.e_hist(obj.h_len,:)=obj.e_last ;
            
            obj.e_hist(obj.h_len, :)=e';
            obj.tau_hist(obj.h_len, :)=tau;
            obj.t_hist(obj.h_len)=t;
            obj.h_len=obj.h_len+1;
        end
        
        
        function [obj,tau] = PIDman(obj, U, t, dt)
            % Given a reference velocity command U=[q1dot,q2dot],
            % generate a torque command tau using PID control.

            
            e = [0,0]; % proportional error
            
            de=[0,0]; % derivitave error

            e_hist = obj.U_hist - obj.V_hist;
           
            obj.samples = 0;
            obj.U_hist=zeros(20,2);
            obj.V_hist=zeros(20,2);
            E=e_hist(1,:); % integral error
            
            
            % Epproximate de/dt by averaging error gradient since last
            % control command was sent.
            % And approximate integral error by summing errors and deviding
            % by time past 
            for i=2:10
                de_this = (e_hist(i,:) - e_hist(i-1,:))./dt;
                if i ~= 10
                    E_this = e_hist(i,:).*2;
                else
                    E_this = e_hist(i,:); % Trapizoidal rule
                end
                e_this = e_hist(i,:) + e_hist(i-1,:);
                
                
                de = de + de_this;
                E = E + E_this;
                e_this = e + e_this;
            end
            de = de./10;
            E = dt.*E./2;
            e = e_this./10;
            

            
            % now we have estimated e, E, and de, calculate control effort
            tau = obj.K.*(obj.K_p*e' + obj.K_d*de' + obj.K_i*E');
            
            % Store errors and torque
            obj.e_hist(:,obj.h_len)=e';
            obj.E_hist(:,obj.h_len)=E';
            obj.de_hist(:,obj.h_len)=de';
            obj.tau_hist(:,obj.h_len)=tau;
            obj.t_hist(obj.h_len)=t;
            
            obj.h_len = obj.h_len+1;
        end     
        
        function [obj, X] = HAX(obj, X0,t,dt)
        % Robot bypasses ODE45 entirely to perfectly follow a trajectory
            X=X0;
            obj.state=obj.state.updateState(X, t+dt);
            
        end
            
        
        
        function plotError(obj)
            r2d=360/(2*pi);
            figure()
            hold on
            plot(obj.t_hist, r2d.*obj.e_hist(:,1));
            plot(obj.t_hist, r2d.*obj.e_hist(:,2));
%             plot(obj.t_hist, r2d.*obj.E_hist(1,:));
%             plot(obj.t_hist, r2d.*obj.de_hist(1,:));
            
            xlabel('time (s)');
            ylabel('error (deg)');
            legend('e1','e2');
            grid();
            
        end
        
        
        
        function plotError1(obj)
            r2d=360/(2*pi);
            figure()
            hold on
            plot(obj.t_hist, r2d.*obj.e_hist(:,1));
%             plot(obj.t_hist, r2d.*obj.E_hist(1,:));
%             plot(obj.t_hist, r2d.*obj.de_hist(1,:));
            
            
            legend('e1');
            
        end
        
        function plotError2(obj)
            r2d=360/(2*pi);
            figure()
            hold on
            plot(obj.t_hist, r2d.*obj.e_hist(:,1));
%             plot(obj.t_hist, r2d.*obj.E_hist(2,:));
%             plot(obj.t_hist, r2d.*obj.de_hist(2,:));
            
            legend('e2');
        end
        
        function plotTau(obj, alone)
            r2d=360/(2*pi);
            if alone
                figure()
                hold on
                legend('tau1','tau2');
            end
            plot(obj.t_hist, r2d.*obj.tau_hist(:,1),'-','color','k');
            plot(obj.t_hist, r2d.*obj.tau_hist(:,2),'-','color','r');

            if ~alone
               legend('e1','e2','tau1','tau2'); 
            end
        end
        
        
    end
end