classdef RevoluteJoint < ActuatedJoint
    %REVOLUTEJOINT Revolute-Joint definition.
    %   This is a primitive joint class, characterised by a single 
    %   (rotational) degree of freedom.

    properties (Constant)
        DegreesOfFreedom = 1;
        NumberOfInputs = 1;
    end
    
    methods
        function [this] = RevoluteJoint()
            %REVOLUTEJOINT Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
end

