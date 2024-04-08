classdef Interval < Boundary
    % A 1D boundary constraint
    
    properties (Constant)
        Code = ColliderCode.Line;
    end
    properties
        Min = -1;
        Max = 1;
    end
    methods
        function [this] = Interval(min,max)
            % CONSTRUCTOR - Create an instance of the interval class.
            
            if nargin > 1
                this.Max = max;
            end
            if nargin > 0
                if length(min) > 1
                    this.Min = min(1);
                    this.Max = min(2);
                else
                    this.Min = min;
                end
            end
        end
        % Operators
        function [r] = plus(obj1,obj2)
            % How addition is done with intervals.
            assert(isscalar(obj1),"Must be scalar.");
            assert(isscalar(obj2),"Must be scalar.");
            
            if isnumeric(obj1) 
                obj1 = double(obj1);
                r = Interval(obj2.Min + obj1,obj2.Max + obj1);
                return;
            end

            if isnumeric(obj2)
                obj2 = double(obj2);
                r = Interval(obj1.Min + obj2,obj1.Max + obj2);
                return;
            end

            if isa(obj1,"Interval") && isa(obj2,"Interval")
                r = Interval(obj1.Min + obj2.Min,obj1.Max + obj2.Max);
            end
        end
        function [r] = minus(obj1,obj2)
            % How addition is done with intervals.
            assert(isscalar(obj1),"Must be scalar.");
            assert(isscalar(obj2),"Must be scalar.");
            
            if isnumeric(obj1) 
                obj1 = double(obj1);
                r = Interval(obj2.Min - obj1,obj2.Max - obj1);
                return;
            end

            if isnumeric(obj2)
                obj2 = double(obj2);
                r = Interval(obj1.Min - obj2,obj1.Max - obj2);
                return;
            end

            if isa(obj1,"Interval") && isa(obj2,"Interval")
                r = Interval(obj1.Min - obj2.Max,obj1.Max - obj2.Min);
            end
        end
        function [r] = mtimes(obj1,obj2)
            % How addition is done with intervals.
            if isa(obj1,"Interval")
                assert(isnumeric(obj2),"Expecting a second numeric input.");
                r = Interval(obj2*obj1.Min,obj2*obj1.Max);
                return;
            end

            if isa(obj2,"Interval")
                assert(isnumeric(obj1),"Expecting a first numeric input.");
                r = Interval(obj1 * obj2.Min,obj1*obj2.Max);
                return;
            end
        end
    end

    %% Instance Operations
    methods
        function [flag,t_enter,t_exit] = IntersectRay(this,ray_origin,ray_direction,t_enter,t_exit)
            % Test a ray and this intervale for intersection. 
            % origin - The start of the ray in this intervals dimension.
            % direction - The direction of the ray in this intervals dimension.
  
            flag = false;

            % Minimal direction count
            if abs(ray_direction) < 0.000001
                flag = true;
                return;    
            end
            % Compute the intersection params   
            % float u0, u1;    
            u0 = (this.Min - ray_origin) / (ray_direction);   
            u1 = (this.Max - ray_origin) / (ray_direction);    
            % sort them (t1 must be >= t0)    
            if u0 > u1        
                % Swap the coordinates if the times are backward
                a = u0;
                u0 = u1;
                u1 = a;
            end

            % Check if the intersection is within the ray range    
            if (u1 < t_enter || u0 > t_exit)       
                % Update the ray range
                return;    
            end   
            t_enter = max(u0, t_enter);    
            t_exit  = min(u1, t_exit);    
            % Ah. we missed the interval    
            flag = t_exit >= t_enter;
        end
    end
    
    %% Static Operations
    methods (Static)
        function [flag] = Contains(first,second)
            % Test if the interval contains a coordinate
            flag = true;
            if first.Min > second || first.Max < second
                flag = false;
            end
        end
        function [flag] = IsIntersecting(first,second)
            % Test two intervals for intersection
            flag = true;
            if first.Min > second.Max || first.Max < second.Min
                flag = false;
            end
        end   
    end
end