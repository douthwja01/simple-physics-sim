classdef FloatingJoint < MovableJoint
    %FLOATINGJOINT Floating-Joint definition.
    %   This is primitive joint class floating-joint definition, 
    %   characterised by its six degrees of freedom.
    
    properties (Constant)
        Code = JointCode.Floating;
        DegreesOfFreedom = 6;
    end
    
    methods
        function [this] = FloatingJoint()
            %FLOATINGJOINT Construct an instance of this class
            %   This is a joint definition for a free-floating 6-DOF joint.

            % Call the parent class
            [this] = this@MovableJoint();
        end
        % Get/sets
        function [S] = GetMotionSubspace(this)
            % Get the motion subspace matrix for this joint.
            S = eye(this.DegreesOfFreedom);
        end
        function [T] = GetConstraintSubspace(this)
            % Get the constraint subspace matrix for this joint.
            T = [];                                                         % Constrained in no dimensions
        end
    end
end

