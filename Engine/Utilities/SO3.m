%% A simple Transform Class (Transform.m) %%%%%%%%%%%%%%%%%%%%
% A class define a container describing the pose of a reference frame.
% This class is only responsible for the description of an objects
% static position and orientation. 

classdef SO3 < handle
    properties (SetObservable = true,AbortSet)
        Position = zeros(3,1);
        Rotation = Quaternion();
        Scale = ones(3,1);
    end

    methods
        % Constructor
        function [this] = SO3(p,q)
            % CONSTRUCTOR - Creates and instance of the 'Transform' class
            % from an initial Cartesian position [3x1] and Quaternion
            % [4x1].

            % Input parsing
            if nargin < 2
                q = Quaternion();
            end
            if nargin < 1
                p = zeros(3,1);
            end
            this.Position = p;
            this.Rotation = q;

            % Listen for future changes
            addlistener(this,"Position","PostSet",@(src,evnt)OnTransformUpdate(this));
            addlistener(this,"Rotation","PostSet",@(src,evnt)OnTransformUpdate(this));
        end
        % Get/sets
        function set.Position(this,p)
            assert(IsColumn(p,3),"Expecting a valid Cartisian position [3x1].");
            this.Position = p;
        end
        function set.Rotation(this,q)
            assert(isa(q,"Quaternion"),"Expecting a valid Quaternion [4x1].");
            this.Rotation = q;
        end
        function set.Scale(this,s)
            assert(IsColumn(s,3),"Expecting a valid 3D scale vector [3x1].");
            this.Scale = s;
        end   
    end

    %% Further accessors
    methods
        % The scale matrix
        function [Ts] = GetScaleMatrix(this)
            % Get the matrix representing the scale assigned to this SO3.
            s = this.Scale;
            % Create the scaling matrix
            Ts = [s(1),0,0,0; 0,s(2),0,0; 0,0,s(3),0; 0,0,0,1];
        end
        % Euler Angles
        function [phi,theta,psi] = GetEulers(this)
            % This function returns the current set of euler angles.
            [phi,theta,psi] = PhysicsExtensions.QuaternionToEulers( ...
                this.Rotation);
        end
        function [this] = SetEulerAngles(this,phi,theta,psi)
            % This function assigns a given set of euler angles.
            this.Rotation = PhysicsExtensions.EulersToQuaternion( ...
                phi,theta,psi);
        end
        % Rotation matrices
        function [R] = GetRotationMatrix(this)
            % This function returns the current rotation matrix.
            R = PhysicsExtensions.QuaternionToRotation( ...
                this.Rotation);
        end
        function [this] = SetRotationMatrix(this,R)
            % This function assigns a given rotation matrix.
            this.Rotation = PhysicsExtensions.RotationToQuaternion(R);
        end
        % Transformation matrices
        function [T] = GetMatrix(this)
            % This function returns the current transformation matrix.
            Tq = PhysicsExtensions.QuaternionTransform( ...
                this.Position, ...
                this.Rotation);
            Ts = this.GetScaleMatrix();
            T = Tq*Ts;
        end
        function [this] = SetMatrix(this,T)
            % This function assigns a given transformation matrix.
            [this.Position,R] = PhysicsExtensions.TransformToPose(T);
            this.Rotation = PhysicsExtensions.RotationToQuaternion(R);
            % [how to extract scale?]
        end
        % Vector States
        function [state] = GetState(this)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.
            state = [this.Rotation;this.Position];
        end
        function [this] = SetState(this,state)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.

            assert(numel(state) == 7,"Expecting a valid state vector [7x1].");
            % Assign the values
            this.Rotation = state(1:4,1);
            this.Position = state(5:7,1);
        end
    end

    %% Support functions
    methods 
        function [this] = Zero(this)
            % Reset to default transform
            this.Rotation = Quaternion();
            this.Position = zeros(3,1);
        end
        function [flag] = IsSymbolic(this)
            % A simple check if the transform is symbolically defined.
            flag = isa(this.Position,"sym") || isa(this.Rotation,"sym");
        end
        function [hand] = Plot(this,container)
            % Plot a given transform to a given container or default to the
            % current axes.

            % default to current figure
            if nargin < 2
                container = gca;
            end

            % Create transform
            hand = hgtransform( ...
                container, ...
                "DisplayName","Transform",...
                "Tag","Transform");

            % Get the gizmo appearance properties
            gizmoParams = Graphics.GizmoProperties();

            % Return if no numerics
            if this.IsSymbolic()
                warning("Unable to plot SO3 group as its transform is symbolic, drawing identity.");
                set(hand,"Matrix",eye(4));
            else
                % Attempt to get numeric instance of this transform
                set(hand,"Matrix",this.GetMatrix());
            end

            % Draw a triad at the location
            hTri = Graphics.DrawTriad(gizmoParams.scale);
            set(hTri,'Parent',hand);
        end
    end

    %% Internals
    methods (Access = private)
        function [this] = OnTransformUpdate(this)
            % This event is notified when the transform value is updated.
            notify(this,"TransformChanged");
        end
    end
    events (NotifyAccess = private)
        TransformChanged;
    end
end