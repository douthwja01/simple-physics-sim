classdef PrismaticJoint < ActuatedJoint
    %PRISMATICJOINT Prismatic-Joint definition.
    %   This is a primitive joint class, characterised by a single 
    %   (translational) degree of freedom.
    
    properties (Constant)
        Code = JointCode.Prismatic;
        DegreesOfFreedom = 1;
        NumberOfInputs = 1;
    end
    properties
        Axis = AxisCode.Z;
    end

    methods
        function [this] = PrismaticJoint()
            %PRISMATICJOINT Construct an instance of this class
            
            % Call the parent class
            [this] = this@ActuatedJoint();
        end
        % Get/sets
        function [S] = GetMotionSubspace(this)
            % Get the motion subspace matrix for this joint.
            a = AxisCode.GetVector(this.Axis);
            S = [0;0;0;a];
        end
        function [T] = GetConstraintSubspace(this)
            % Get the constraint subspace matrix for this joint.
            a = ~AxisCode.GetVector(this.Axis);
            % Logically select subspace
            e = eye(3);
            mat = e(:,a);
            % Construct the constraint matrix
            T = [eye(3),zeros(3,2);zeros(3),mat];
        end
    end
    methods (Access = protected)
        function [this] = OnJointPositionUpdate(this)
            % On update of the joint position variable.
            switch this.Axis
                case AxisCode.X
                    this.Pivot.Position = [this.JointPosition;0;0];
                case AxisCode.Y
                    this.Pivot.Position = [0;this.JointPosition;0];
                case AxisCode.Z
                    this.Pivot.Position = [0;0;this.JointPosition];
                otherwise
                    error("Axis code not applicable.");
            end
        end
    end
end

