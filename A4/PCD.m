classdef PCD
    % Point cloud data for estimating where an obsticle is from a bunch of
    % points.
    properties
        min_points= 5 % minimum number of points before fitting a circle
        circ=CircleE();
        n=0;        % number of points in the cloud
        points={};  % array of points
        
    end
    methods
        function obj=PCD()
            
        end
        
        function obj=update(obj, x, y)
           % adds a point to the cloud and updates relevant properties 
            obj.n = obj.n+1;
            obj.points(obj.n) = {[x,y]};
            
            % if we dont have many points do a basic estimate of radius and
            % centre
            cx=0;
            cy=0;
            for i=1:obj.n
                cx = cx + obj.points(i).x;
                cy = cy + obj.points(i).y;
            end
            cx = x/obj.n;
            cy = y/obj.n;

            if obj.n < obj.min_points
                obj.c = obj.com;
                obj.r = obj.max();
                
            else
                
            % if there are enough points fit a circle to them
                
                obj.circ.update(obj.points);
            end
        end
        
        
        
        function c = isBlocked(obj, X, tf)
            % given trajectory X(t)=(x(t),y(t)), if the circle is expected
            % to block the path, return it, otherwise return 0
            
            for t=0:0.01:tf
                x = X(t);
                y = Y(t);
                c = obj.circ.isContained(x,y);
            end
        end
        
        function d = d2p(p1,p2)
           d=sqrt((p1(1) + p2(1))^2 + (p1(2) + p2(2))^2);
        end
        
        function d = max(obj)
            % if not many points are collected yet, will just assume the
            % radius is the max distance between any two points
            d=0;
            for i=1:obj.n
                p1=obj.points(i,:);
                
                for j=(i+1:obj.n)
                    p2 = obj.points(j,:);
                    d_ = obj.d2p(p1,p2);
                    if d_ > d
                        d = d_;
                    end
                end
            end
        end
        
    end
end





