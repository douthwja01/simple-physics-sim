classdef Interval
    properties
        Min = 0;
        Max = 1;
    end
    properties (Dependent)
        Range;
    end
    methods
        function [this] = Interval(min,max)
            if nargin > 1
                this.Min = min;
                this.Max = max;
            elseif nargin > 0
                assert(numel(min) == 2,"Expecting a valid vector [min,max].");
                this.Min = min(1);
                this.Max = min(2);
            end
        end
        % Get/sets
        function [r] = get.Range(this)
            r = [this.Min,this.Max];
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
    methods (Static)
        function [flag] = TestIntersection(a,b)
            % Test two intervals for intersection
            flag = true;
            if a.Min > b.Max || a.Max < b.Min
                flag = false;
            end
        end
    end
end