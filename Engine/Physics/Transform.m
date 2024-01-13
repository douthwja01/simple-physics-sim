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
        Velocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
        Acceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
    end
    properties (Hidden)
        PriorPosition = [];
    end
    properties (Dependent)
        %         rotation;
        %         position;
        %         state;
        %         IsSymbolic;
    end
    properties
        IsStatic = false;
    end
    methods
        % Constructor
        function [this] = Transform(p,q)
            % CONSTRUCTOR - Creates and instance of the 'Transform' class
            % from an initial Cartesian position [3x1] and Quaternion
            % [4x1].

            % Input checks
            if nargin < 2
                q = [1;0;0;0];
            end
            if nargin < 1
                p = zeros(3,1);
            end

            % Assign initial values
            this.Position = p;
            this.Quaternion = q;
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
    end

    %% Support functions
    methods
        function [R] = GetRotation(this)
            
        end
        function [T] = GetMatrix(this)
            
        end
        function [state] = GetState(this)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.
            state = [this.Quaternion;this.Position];
        end
        function [state] = SetState(this,state)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.

            assert(numel(state) == 7,"Expecting a valid state vector [7x1].");
            % Assign the values
            this.Quaternion = state(1:4,1);
            this.Position = state(5:7,1);
        end
        function [this] = Zero(this)
            % Reset to default transform
            this.Quaternion = [1;0;0;0];
            this.Position = zeros(3,1);
        end
        % Other
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
                m = this.Matrix * Transform.Scale(this.Scale);
                set(hand,"Matrix",m);
            end

            % Draw a triad at the location
            hTri = Graphics.DrawTriad(gizmoParams.scale);
            set(hTri,'Parent',hand);
        end
    end

    %% Tools (Instance)
    methods
        function [s] = WorldScale(this)
            % This scale multiplied by all the parent scales

            % [To fix after parentage]
            s = this.Scale;
        end
        function [p] = WorldPosition(this)
            % This transform multiplied by all its parents

            % [To fix after parentage]
            p = this.position;
        end
        % Rotation conventions
        function [eta] = GetEulers(this)
            % Get the equivalent euler rotations
            eta = Euler.FromRotationMatrix(this.rotation);
        end
        function [this] = SetEulers(this,eta)
            % Set the rotation by euler rotations
            this.rotation = Eulers.ToRotation(eta);
        end
        function [q] = GetQuaternion(this)
            % Get the equivalent quaternion rotation
            q = Quaternion.FromRotation(this.rotation);
        end
        function [this] = SetQuaternion(this,q)
            % Set the transform via a quaternion
            this.rotation = Quaternion.ToRotation(q);
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