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
            if nargin > 0
                if IsColumn(p,3)
                    this.Position = zeros(3,1);
                else
                    this.SetMatrix(p);
                end
            end
            if nargin > 1
                this.Rotation = q;
            end

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

    %% (Instance) Methods
    methods
        % Transformation matrices
        function [T] = ToMatrix(this)
            % This function returns the current transformation matrix.

            % Extract the state
            Rq = this.ToRotationMatrix();
            % Create the matrix
            Tq = [Rq,this.Position;zeros(1,3),1];
            % The scale matrix
            Ts = this.ToScaleMatrix();
            % Combine
            T = Tq*Ts;
        end
        function [Ts] = ToScaleMatrix(this)
            % Get the matrix representing the scale assigned to this SO3.
            s = this.Scale;
            % Create the scaling matrix
            Ts = [s(1),0,0,0; 0,s(2),0,0; 0,0,s(3),0; 0,0,0,1];
        end
        % Euler Angles
        function [phi,theta,psi] = ToEulerAngles(this)
            % This function returns the current set of euler angles.
            [phi,theta,psi] = this.Rotation.ToEulers();
        end
        function [this] = SetEulerAngles(this,phi,theta,psi)
            % This function assigns a given set of euler angles.
            this.Rotation = Quaternion.FromEulers(phi,theta,psi);
        end
        % Rotation matrices
        function [R] = ToRotationMatrix(this)
            % This function returns the current rotation matrix.
            R = this.Rotation.ToRotationMatrix();
        end
        function [this] = SetRotationMatrix(this,R)
            % This function assigns a given rotation matrix.
            this.Rotation = Quaternion.FromRotationMatrix(R);
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
        % General
        function [T] = DirectionOnly(this)
            % Extract only the rotational components
            T = SO3(zeros(3,1),this.ToRotationMatrix());
        end
        function [T] = TranslationOnly(this)
            % Extract only the translational components
            T = SO3(this.Position);
        end
        function [this] = Normalise(this)
            % This function normalises the transform representation.
            
            % Get the matrix
            this.SetMatrix(NormaliseMatrix(this.ToMatrix()));
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
                set(hand,"Matrix",this.ToMatrix());
            end

            % Draw a triad at the location
            hTri = Graphics.DrawTriad(gizmoParams.scale);
            set(hTri,'Parent',hand);
        end
    end

    %% (Static) Support methods
    methods (Static)
        % Matrix handling
        function [T] = FromMatrix(tMatrix)
            % Transform -> Position & Rotation
            assert(IsSquare(tMatrix,4),"Expecting a [4x4] transformation matrix.");
            % Extract the matrix components
            T = SO3(tMatrix(4,1:3),tMatrix(1:3,1:3));
        end
        % Creation
        function [T] = FromMDHParameters(d,theta,a,alpha)
            % A Modified Denavit-Hartenberg transform
            % This difference between the classical 'dh' and the modified 'mdh'
            % transformation matrix is the position of the coordinate system. Under the
            % modified representation, the coordinates of Oi is put on the axis 'i',
            % not the axis 'i+1' in the classic DH convention.

            % Inputs
            % d     - The displacement of Oi along axis Zi.
            % theta - The rotation of Oi about axis Zi.
            % a     - The radius of rotation of link i to i+1.
            % alpha - The angle made between axis 'zi' and 'zi+1'

            % Sanity check
            assert(isscalar(theta),"Expecting a scalar angle between zi and zi+1.");
            assert(isscalar(a),"Expecting a scalar rotation of i to i+1.");
            assert(isscalar(d),"Expecting a scalar distance along z_i.");
            assert(isscalar(alpha),"Expecting a scalar angle between zi and zi+1.");

            % Geometric projections
            sa = sin(alpha);
            ca = cos(alpha);
            st = sin(theta);
            ct = cos(theta);

            % modified DH
            TMatrix = [...
                ct,     -st,    0,      a;
                st*ca,   ct*ca,   -sa, -sa*d;
                st*sa,   ct*sa,    ca,  ca*d;
                0,       0,     0,     1];
            % Return the SO3
            T = SO3(TMatrix);
        end
        function [T] = FromDHParameters(theta,a,d,alpha)
            % A Denavit-Hartenberg transform
            % INPUTS:
            % d     - Offset along previous z to the common normal
            % theta - Angle about previous z, from the old x to new x
            % r/a   - Length of the common normal (radius about previous z)
            % alpha - Angle about common normal from old z-axis to new z-axis.

            % Sanity check
            assert(isscalar(theta),"Expecting a scalar angle around previous axis.");
            assert(isscalar(a),"Expecting a scalar common radial distance.");
            assert(isscalar(d),"Expecting a scalar offset along previous normal.");
            assert(isscalar(alpha),"Expecting a scalar normal distnace betwee z-axes.");

            % Geometric projections
            sa = sin(alpha);
            ca = cos(alpha);
            st = sin(theta);
            ct = cos(theta);
            % Standard DH transform
            TMatrix = [...
                ct, -st*ca,   st*sa, a*ct;
                st,  ct*ca,  -ct*sa, a*st;
                0,     sa,      ca,    d;
                0,      0,       0,    1];
            % Return the SO3
            T = SO3(TMatrix);
        end
        function [T] = FromScale(s)
            % This function creates a transformation matrix that preforms a scaling
            % multiplication by the value(s) provided.

            % Sanity check
            assert(isnumeric(s) || isa(s,"sym"),"Expecting either a numeric or symbolic scaling variable(s)");
            assert(numel(s) == 1 || numel(s) == 3,"Expecting either a scalar or axis-wise scaling input.");

            % Sanity Check
            switch numel(s)
                case 1
                    S = ones(3,1)*s;
                case 3
                    S = s;
                otherwise
                    error("Expecting either a scalar or [3x1] vector of scaling values.");
            end
            % Create the scaling matrix
            T = SO3([S(1),0,0,0; 0,S(2),0,0; 0,0,S(3),0; 0,0,0,1]);
        end
        function [T] = FromComponents(x,y,z,rx,ry,rz)
            % Axis displacements -> Transform
            % Sanity check
            assert(isscalar(x),"Expecting a scalar x displacement.");
            assert(isscalar(y),"Expecting a scalar y displacement.");
            assert(isscalar(z),"Expecting a scalar z displacement.");
            assert(isscalar(rx),"Expecting a scalar x rotation.");
            assert(isscalar(ry),"Expecting a scalar y rotation.");
            assert(isscalar(rz),"Expecting a scalar z rotation.");
            % Compound transform
            T = SO3([x;y;z],Quaternion.FromEulers(rx,ry,rz));
        end
        % General
        function [T] = Symbolic()
            % Generate a symbolic version of the transform.
            s = sym("q%d",[6,1],"real");
            T = SO3.FromScalars(s(1),s(2),s(3),s(4),s(5),s(6));
        end
        function [T] = Zero()
            % Reset to default transform
            T = SO3(zeros(3,1),Quaternion());
        end
    end

    %% Internals
    methods (Access = private)
        function [T] = NormaliseMatrix(T)
            % This function normalises the transform representation.
            
            assert(IsSquare(T,4),"Expecting a valid 4x4 transform matrix.");
            
            R = Transform.ToRotation(T);
            p = Transform.ToPosition(T);
            j = R(1:3,2);
            k = R(1:3,3);
            % Recalculate perpendicular assets
            i = cross(j, k);         % N = O x A
            j = cross(k, i);         % O = A x N
            % Reassign rotation
            T = Transform.FromPose(p,[Unit(i),Unit(j),Unit(k)]);
        end
        function [this] = OnTransformUpdate(this)
            % This event is notified when the transform value is updated.
            notify(this,"TransformChanged");
        end
    end
    events (NotifyAccess = private)
        TransformChanged;
    end
end