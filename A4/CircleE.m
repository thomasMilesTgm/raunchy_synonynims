classdef CircleE
    properties
        a; % x coordinate of centre
        b; % y coordinate of centre
        r;% radius
        
    end
    methods
        function obj=Circle(~)
            obj.a=1000;
            obj.b=1000;
            obj.r=1e-10;
        end
        
        function obj=update(obj, points)
            [obj.a, obj.b, obj.r] = CircleFitByPratt(points);
        end
        
        function c=isContained(obj,x,y)
            % takes a trajectory and checks if it crosses a circle
            c=0;
            d=sqrt((x-obj.a)^2+(y-obj.b)^2);
            if d < obj.r
               c = obj;
            end
        end
    end
end