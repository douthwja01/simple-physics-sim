
classdef (Abstract) Boundary < handle
    % A primitive defining a boundary
    properties (Abstract,Constant)
        Code;
    end
    properties (Abstract)
        Min;
        Max;
    end  
    properties (Dependent)
        Range;
        Span;
    end

    methods
        function [range] = get.Range(this)
            % Return the vector defining the boundaries range extents.
            range = [this.Min,this.Max];
        end
        function [span] = get.Span(this)
            % Return the magnitude between each dimensional boundary.
            span = this.Max - this.Min;
        end
        function [v] = GetMidPoint(this)
            % Calculate the mid-point (center) of the boundary from its
            % extents.
            v = this.Min + this.Span./2;
        end
    end
end