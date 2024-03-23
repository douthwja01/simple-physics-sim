classdef (Abstract) MovableJoint < Joint
    %MOVABLEJOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        JointPosition = 0;
        JointVelocity = 0;
        JointAcceleration = 0;
    end
    
    methods
        function [this] = MovableJoint()
            %MOVABLEJOINT Construct an instance of this class
            %   Detailed explanation goes here

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

