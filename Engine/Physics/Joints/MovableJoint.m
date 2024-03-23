classdef (Abstract) MovableJoint < Joint
    %MOVABLEJOINT Movable-Joint definition.
    %   This is a base class for all movable-joint definitions.
    
    properties (Abstract, Constant)
        DegreesOfFreedom;
    end
    properties
        JointPosition = double.empty;
        JointVelocity = double.empty;
        JointAcceleration = double.empty;
    end
    
    methods
        function [this] = MovableJoint()
            %MOVABLEJOINT Construct an instance of this class
            %   This is a base class for all movable-joint definitions.

            % Call the parent class
            [this] = this@Joint();
            % Default joint states
            this.JointPosition = zeros(this.DegreesOfFreedom,1);
            this.JointVelocity = zeros(this.DegreesOfFreedom,1);
            this.JointAcceleration = zeros(this.DegreesOfFreedom,1);
        end
        % Get/sets
        function set.JointPosition(this,p)
            assert(isnumeric(p),"Expecting a numeric joint position.");
            this.JointPosition = p;
        end
        function set.JointVelocity(this,v)
            assert(isnumeric(v),"Expecting a numeric joint velocity.");
            this.JointVelocity = v;
        end
        function set.JointAcceleration(this,a)
            assert(isnumeric(a),"Expecting a numeric joint acceleration.");
            this.JointAcceleration = a;
        end
    end
end

