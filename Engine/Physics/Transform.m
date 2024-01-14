%% A simple Transform Class (Transform.m) %%%%%%%%%%%%%%%%%%%%
% A class define a container describing the motion and pose of a point of
% reference or frame.

classdef Transform < Element
    properties (SetObservable = true,AbortSet)
        Position = zeros(3,1);
        Quaternion = [1;0;0;0];
        Scale = ones(3,1);
    end
    % Kinematic properties (simple containers)
    properties
        IsStatic = false;
        Velocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
        Acceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
    end
    properties (Hidden)
        PriorPosition = [];
    end
    methods
        % Constructor
        function [this] = Transform(entity)
            % CONSTRUCTOR - Creates and instance of the 'Transform' class
            % from an initial Cartesian position [3x1] and Quaternion
            % [4x1].

            % Assign the entity
            [this] = this@Element(entity);
            % Listen for future changes
            addlistener(this,"Position","PostSet",@(src,evnt)OnTransformUpdate(this));
            addlistener(this,"Quaternion","PostSet",@(src,evnt)OnTransformUpdate(this));
        end
        % Get/sets
        function set.Position(this,p)
            assert(IsColumn(p,3),"Expecting a valid Cartisian position [3x1].");
            this.Position = p;
        end
        function set.Quaternion(this,q)
            assert(IsColumn(q,4),"Expecting a valid Quaternion [4x1].");
            this.Quaternion = q;
        end
        function set.Scale(this,s)
            assert(IsColumn(s,3),"Expecting a valid 3D scale vector [3x1].");
            this.Scale = s;
        end   
        function set.IsStatic(this,s)
            assert(islogical(s),"Expecting a valid logical IsStatic flag.");
            this.IsStatic = s;
        end
        function set.Velocity(this,v)
            assert(IsColumn(v,3),"Expecting a valid Cartesian linear velocity [3x1].");
            this.Velocity = v;
        end
        function set.AngularVelocity(this,w)
            assert(IsColumn(w,3),"Expecting a valid Cartesian angular velocity [3x1].");
            this.AngularVelocity = w;
        end 
        function set.Acceleration(this,dv)
            assert(IsColumn(dv,3),"Expecting a valid Cartesian linear acceleration [3x1].");
            this.Acceleration = dv;
        end  
        function set.AngularAcceleration(this,dw)
            assert(IsColumn(dw,3),"Expecting a valid Cartesian angular acceleration [3x1].");
            this.AngularAcceleration = dw;
        end 
    end

    %% Further accessors
    methods
        % Parentage
        function [s] = WorldScale(this)
            % This scale multiplied by all the parent scales

            % [To fix after parentage]
            s = this.Scale;
        end
        function [p] = WorldPosition(this)
            % This transform multiplied by all its parents

            % [To fix after parentage]
            p = this.Position;
        end
        % Numeric representation
        function [phi,theta,psi] = GetEulers(this)
            % This function returns the current set of euler angles.
            [phi,theta,psi] = PhysicsExtensions.QuaternionToEulers( ...
                this.Quaternion);
        end
        function [this] = SetEulers(this,phi,theta,psi)
            % This function assigns a given set of euler angles.
            this.Quaternion = PhysicsExtensions.EulersToQuaternion( ...
                phi,theta,psi);
        end
        function [R] = GetRotation(this)
            % This function returns the current rotation matrix.
            R = PhysicsExtensions.QuaternionToRotation( ...
                this.Quaternion);
        end
        function [this] = SetRotation(this,R)
            % This function assigns a given rotation matrix.
            this.Quaternion = PhysicsExtensions.RotationToQuaternion(R);
        end
        function [T] = GetMatrix(this)
            % This function returns the current transformation matrix.
            Tq = PhysicsExtensions.QuaternionTransform( ...
                this.Position, ...
                this.Quaternion);
            Ts = PhysicsExtensions.ScaleTransform( ...
                this.Scale);
            T = Tq*Ts;
        end
        function [this] = SetMatrix(this,T)
            % This function assigns a given transformation matrix.
            [this.Position,R] = PhysicsExtensions.TransformToPose(T);
            this.Quaternion = PhysicsExtensions.RotationToQuaternion(R);
        end
        function [state] = GetState(this)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.
            state = [this.Quaternion;this.Position];
        end
        function [this] = SetState(this,state)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.

            assert(numel(state) == 7,"Expecting a valid state vector [7x1].");
            % Assign the values
            this.Quaternion = state(1:4,1);
            this.Position = state(5:7,1);
        end
    end

    %% Support functions
    methods 
        function [this] = Zero(this)
            % Reset to default transform
            this.Quaternion = [1;0;0;0];
            this.Position = zeros(3,1);
        end
        function [flag] = IsSymbolic(this)
            % A simple check if the transform is symbolically defined.
            flag = isa(this.Position,"sym") || isa(this.Quaternion,"sym");
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