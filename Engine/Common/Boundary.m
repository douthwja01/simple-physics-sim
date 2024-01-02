
classdef (Abstract) Boundary < handle
    % A primitive defining a boundary
    properties (Abstract,Constant)
        Code;
    end
    properties (Abstract)
        Center;
        Min;
        Max;
        Range;
    end    
end