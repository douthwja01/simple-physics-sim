%% A simple Transform Class (Transform.m) %%%%%%%%%%%%%%%%%%%%
% A class define a container describing the pose of a reference frame.
% This class is only responsible for the description of an objects
% static position and orientation. 

classdef SO3 < handle
    properties
        Position = zeros(3,1);
        Rotation = Quaternion();
        Scale = ones(3,1);
    end
    properties (Hidden)
        HasChanged = false;
    end

    methods
        % Constructor
        function [this] = SO3(p,q)
            % CONSTRUCTOR - Creates and instance of the 'Transform' class
            % from an initial Cartesian position [3x1] and Quaternion
            % [4x1].

            % Input parsing
            if nargin < 1
               p = zeros(3,1);
            end
            if nargin < 2
                q = Quaternion.Zero;
            end
            % Assign initial properties
            this.Position = p;
            this.Rotation = q;
            this.HasChanged = false; % Ignore initial assignments
        end
        % Get/sets
        function set.Position(this,p)
            assert(IsColumn(p,3),"Expecting a valid Cartisian position [3x1].");
            this.Position = p;
            this.HasChanged = true;
        end
        function set.Rotation(this,q)
            assert(isa(q,"Quaternion"),"Expecting a valid Quaternion [4x1].");
            this.Rotation = q.Normalise();
            this.HasChanged = true;
        end
        function set.Scale(this,s)
            assert(IsColumn(s,3),"Expecting a valid 3D scale vector [3x1].");
            this.Scale = s;
            this.HasChanged = true;
        end   
    end

    %% (Instance) Methods
    methods
        % Gets
        function [T]    = GetMatrix(this)
            % This function returns the current transformation matrix.

            % Extract the state
            Rq = this.Rotation.GetMatrix();
            % Create the matrix
            Tq = [Rq,this.Position;zeros(1,3),1];
            % The scale matrix
            Ts = this.GetScaleMatrix();
            % Combine
            T = Tq*Ts;
        end
        function [Ts]   = GetScaleMatrix(this)
            % Get the matrix representing the scale assigned to this SO3.
            s = this.Scale;
            % Create the scaling matrix
            Ts = [s(1),0,0,0; 0,s(2),0,0; 0,0,s(3),0; 0,0,0,1];
        end            
        function [state] = GetState(this)
            % Return a representitive state vector sufficient to describe
            % the transform's pose.
            state = [this.Rotation;this.Position];
        end        
        % Sets
        function [this] = SetMatrix(this,T)
            % Allow setting of this SO3's properties from a transformation
            
            % The "length" of the scaled axes
            sX = norm(T(1,1:3));
            sY = norm(T(2,1:3));
            sZ = norm(T(3,1:3));
            this.Scale = [sX;sY;sZ];
            this.Rotation = Quaternion.FromRotationMatrix(T(1:3,1:3));
            this.Position = T(4,1:3)';
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
        function [T]    = DirectionOnly(this)
            % Extract only the rotational components
            T = SO3(zeros(3,1),this.GetMatrix());
        end
        function [T]    = TranslationOnly(this)
            % Extract only the translational components
            T = SO3(this.Position);
        end
        function [this] = Normalise(this)
            % This function normalises the transform representation.
            
            % Get the matrix
            this.SetMatrix(NormaliseMatrix(this.GetMatrix()));
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

    %% (Static) Support methods
    methods (Static)
        % Matrix handling
        function [T] = FromMatrix(tMatrix)
            % Transform -> Position & Rotation
            assert(IsSquare(tMatrix,4),"Expecting a [4x4] transformation matrix.");
            
            % Extract the matrix components
            q = Quaternion.FromRotationMatrix(tMatrix(1:3,1:3));
            T = SO3(tMatrix(4,1:3)',q);
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
            % Normalise the rotation matrix
            Rnorm = NormaliseRotationMatrix(R);
            % Reassign rotation
            T = Transform.FromPose(p,Rnorm);
        end
    end
end