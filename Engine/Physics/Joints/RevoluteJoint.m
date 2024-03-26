classdef RevoluteJoint < ActuatedJoint
    %REVOLUTEJOINT Revolute-Joint definition.
    %   This is a primitive joint class, characterised by a single 
    %   (rotational) degree of freedom.

    properties (Constant)
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
    end
    methods (Access = protected)
        function [this] = OnJointPositionUpdate(this)
            % On update of the joint position variable.
            switch this.Axis
                case AxisCode.X
                    this.Pivot.SetEulers(this.JointPosition,0,0);
                case AxisCode.Y
                    this.Pivot.SetEulers(0,this.JointPosition,0);
                case AxisCode.Z
                    this.Pivot.SetEulers(0,0,this.JointPosition);
                otherwise
                    error("Axis code not applicable.");
            end
        end
    end
end

