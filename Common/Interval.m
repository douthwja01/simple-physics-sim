classdef Interval
    properties
        Min = 0;
        Max = 1;
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