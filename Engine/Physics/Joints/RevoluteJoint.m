classdef RevoluteJoint < ActuatedJoint
    %REVOLUTEJOINT Revolute-Joint definition.
    %   This is a primitive joint class, characterised by a single 
    %   (rotational) degree of freedom.

    properties (Constant)
        Code = JointCode.Revolute;
        DegreesOfFreedom = 1;
        NumberOfInputs = 1;
    end
    properties
        Axis = AxisCode.Z;
    end
    
    methods
        function [this] = RevoluteJoint()
            %REVOLUTEJOINT Construct an instance of this class
            
            % Call the parent class
            [this] = this@ActuatedJoint();
        end
        % Get/sets
        function [S] = GetMotionSubspace(this)
            % Get the motion subspace matrix for this joint.
            a = AxisCode.GetVector(this.Axis);
            S = [a;0;0;0];
        end
        function [T] = GetConstraintSubspace(this)
            % Get the constraint subspace matrix for this joint.
            a = ~AxisCode.GetVector(this.Axis);
            T = [diag(a),zeros(3,2);zeros(3,2),eye(3)];
        end
    end
    methods (Access = protected)
        function [this] = OnJointPositionUpdate(this)
            % This function updates the joint-pivot in response to an
            % update of the joint-position value.

            switch this.Axis
                case AxisCode.X
                    %this.Pivot.SetEulers(this.JointPosition,0,0);
                    R = R_x(this.JointPosition);
                case AxisCode.Y
                    %this.Pivot.SetEulers(0,this.JointPosition,0);
                    R = R_y(this.JointPosition);
                case AxisCode.Z
                    %this.Pivot.SetEulers(0,0,this.JointPosition);
                    R = R_z(this.JointPosition);
                otherwise
                    error("Axis code not applicable.");
            end
            % Set the pivot rotation
            this.Pivot.Rotation = Quaternion.FromRotationMatrix(R);
        end
    end
end

