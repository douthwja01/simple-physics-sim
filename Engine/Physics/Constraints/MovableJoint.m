classdef (Abstract) MovableJoint < Joint
    %MOVABLEJOINT Movable-Joint definition.
    %   This is a base class for all movable-joint definitions.
    
    properties (SetObservable = true)
        JointPosition = double.empty;
        JointVelocity = double.empty;
        JointAcceleration = double.empty;
    end
    properties (SetAccess = private)
        Pivot = SO3.empty;
    end

    methods
        function [this] = MovableJoint()
            %MOVABLEJOINT Construct an instance of this class
            %   This is a base class for all movable-joint definitions.

            % Call the parent class
            [this] = this@Joint();
            % Transform recalculation feedback
            addlistener(this,"JointPosition","PostSet",@(src,evnt)OnJointPositionUpdate(this));
            % Default joint pivot
            this.Pivot = SO3.Identity();
            % Default joint states
            this.JointPosition = zeros(this.DegreesOfFreedom,1);
            this.JointVelocity = zeros(this.DegreesOfFreedom,1);
            this.JointAcceleration = zeros(this.DegreesOfFreedom,1);
        end
        % Get/sets        
        function set.Pivot(this,p)
            assert(isa(p,"SO3"),"Expecting a valid pose object for the joint's motion.");
            this.Pivot = p;
        end
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
        function [T] = GetJointTransformation(this)
            % This function computes the total joint transformation 
            e = this.Pivot.GetMatrix();
            T = this.Location.GetMatrix();  
            % Multiply the location by the pivot                               
            T = T*e;                                                        
        end
    end
    methods (Abstract,Access=protected)
        % On update of the joint position
        [this] = OnJointPositionUpdate(src,evnt);
    end
end

