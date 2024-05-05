classdef FixedJoint < Joint
    %FIXEDJOINT Fixed-Joint definition.
    %   This is a primitive joint class, characterised by its unique
    %   zero degree of freedom configuration.
        
    properties (Constant)
        DegreesOfFreedom = 0;
        Code = JointCode.Fixed;
    end

    methods
        function [this] = FixedJoint()
            %FIXEDJOINT Construct an instance of this class
            
            % Call the parent class
            [this] = this@Joint();
        end
        % Get/sets
        function [S] = GetMotionSubspace(this)
            % Get the motion subspace matrix for this joint.
            S = [];                                                         % The motion matrix is null, no motion dimensions.
        end
        function [T] = GetConstraintSubspace(this)
            % Get the constraint subspace matrix for this joint.
            T = eye(this.DegreesOfFreedom);                                 % Constrained in all dimensions
        end
    end
end

