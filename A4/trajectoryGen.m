function [X, Y] = trajectoryGen(X0, Y0, Xf, Yf, tf, f)
    % TRAJECTORYGEN Generates a cubic reference trajectory from given 
    % initial and final conditions.
    %
    % [X, Y] = TRAJECTORYGEN(X0, Y0, Xf, Yf, tf) returns matlab or symbolic
    % functions of polynomials X(t) and Y(t) which satisfy initial and final
    % position and velocity conditions X0 Xf, and Y0 Yf (these are vectors 
    % [X; dX]). 
    % The trajectory will be linear/affine in the x,y plane.


    t=sym('t');
    a1=sym('a1'); b1=sym('b1'); c1=sym('c1'); d1=sym('d1');
    a2=sym('a2'); b2=sym('b2'); c2=sym('c2'); d2=sym('d2');
    
    vars = [a1, b1, c1, d1, a2, b2, c2, d2];

    X=[t;t]; Y=[t;t]; eqns=[t];
    
    X(1) = a1*t^3 + b1*t^2 + c1*t + d1;
    X(2) = 3*a1*t^2 + 2*b1*t + c1;
    Y(1) = a2*t^3 + b2*t^2 + c2*t + d2;
    Y(2) = 3*a2*t^2 + 2*b2*t + c2;
    
    % Substitute constraints to generate equations
    % Initial conditions
    t=0;
    eqns(1) = X0(1) == subs(X(1), t);
    eqns(2) = X0(2) == subs(X(2), t);
    eqns(3) = Y0(1) == subs(Y(1), t);
    eqns(4) = Y0(2) == subs(Y(2), t);
    % Final conditions
    t=tf;
    eqns(5) = Xf(1) == subs(X(1), t);
    eqns(6) = Xf(2) == subs(X(2), t);
    eqns(7) = Yf(1) == subs(Y(1), t);
    eqns(8) = Yf(2) == subs(Y(2), t);

    [A1,B1,C1,D1,A2,B2,C2,D2] = solve(eqns, vars);
    
    X = vpa(subs(X, vars,[A1,B1,C1,D1,A2,B2,C2,D2]));
    Y = vpa(subs(Y, vars,[A1,B1,C1,D1,A2,B2,C2,D2]));
    
    if f
        X = matlabFunction(X);
        Y = matlabFunction(Y);
        
        %Plot trajectories
        set(0,'defaulttextInterpreter','latex') % LaTex is the only way
        i=1;
        t=0:0.1:5;
        x=zeros(length(t),2);
        y=zeros(length(t),2);
        for t=0:0.1:5
            x(i,:)=X(t);
            y(i,:)=Y(t);
            i=i+1;
        end
        figure()
        hold on
        plot(0:0.1:5,x)
        plot(0:0.1:5,y)

        legend('X','dx','Y','dy');
        figure()
        plot(x(:,1),y(:,1));
        xlabel('x (m)');
        ylabel('y (m)');
    end
end











