classdef (Abstract) ActuatedJoint < MovableJoint
    %ACTUATEDJOINT Actuated joint definition.
    %   This is a base class for all actuated-joint definitions.
    
    properties (Abstract, Constant)
        NumberOfInputs;
    end

    properties
        JointInput = double.empty;
    end
    
    methods
        function [this] = ActuatedJoint()
            %ACTUATEDJOINT Construct an instance of this class
            %   This class is the base of all joints with a none-zero
            %   number of inputs.

            % Call the parent class
            [this] = this@MovableJoint();
            % Default joint values
            this.JointInput = zeros(this.NumberOfInputs,1);
        end
        % Get/sets
        function set.JointInput(this,u)
            assert(isnumeric(u),"Expecting a numeric joint input.");
            this.JointInput = u;
        end        
    end
end

