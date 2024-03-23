classdef (Abstract) ActuatedJoint < MovableJoint
    %ACTUATEDJOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        JointInput = 0;
    end
    
    methods
        function obj = ActuatedJoint()
            %ACTUATEDJOINT Construct an instance of this class
            %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
        end
        % Get/sets
        function set.JointInput(this,u)
            assert(isnumeric(u),"Expecting a numeric joint input.");
            this.JointInput = u;
        end        
    end
end

