%% A simple Transform Class (Transform.m) %%%%%%%%%%%%%%%%%%%%
% A class define a container describing the motion and pose of a point of
% reference or frame.

classdef Transform < Element
    %% Main
    properties (SetObservable = true,AbortSet)
        transform = eye(4);
        scale = ones(3,1);
        % Kinematics
        Velocity = zeros(3,1);
        Acceleration = zeros(3,1);
    end
    properties (Dependent)
        rotation;
        position;
        state;
        IsSymbolic;
    end
    properties
        IsStatic = false;
    end
    methods
        % Constructor
        function [this] = Transform(p,R)

            % Input checks
            if nargin < 2
                R = eye(3);
            end
            if nargin < 1
                p = zeros(3,1);
            end

            addlistener(this,"transform","PostSet",@(src,evnt)OnValueChanged(this));
            
            % Write the initial transform
            this.transform = Transform.FromPose(p,R);
        end
        % Get/sets
        function set.scale(this,s)
            assert(IsColumn(s,3),"Expecting a valid scale vecor [3x1].");
            this.scale = s;
        end
        function [R] = get.rotation(this)
            R = Transform.ToRotation(this.transform);
        end
        function set.rotation(this,R)
            assert(IsRotationMatrix(R),"Expecting a valid rotation matrix");
            this.transform(1:3,1:3) = R;
        end
        function [d] = get.position(this)
            d = Transform.ToPosition(this.transform);
        end
        function set.position(this,d)
            assert(IsColumn(d,3),"Expecting a Cartesian vector [3x1].");
            this.transform(1:3,4) = d;
        end
        function set.transform(this,T)
            assert(IsSquare(T,4),"Expecting a transformation matrix [4x4].");
            this.transform = T;
        end
        function [X] = get.state(this)
            % Get the flattened state representation of this transform
            q = this.GetQuaternion();
            p = this.position;
            X = [q;p];
        end
        function set.state(this,X)
            assert(IsColumn(X,7),"Expecting a valid [7x1] flattened state vector.");
            this.SetQuaternion(X(1:4,1));
            this.position = X(5:7,1);
        end
        function [flag] = get.IsSymbolic(this)
            flag = isa(this.transform,"sym");
        end
    end
    
    %% Tools (Instance)
    methods 
        function [th] = Plot(this,container)
            % Plot a given transform to a given container or default to the
            % current axes.
            
            % default to current figure
            if nargin < 2
                container = gca;
            end
            
            % Create transform
            th = hgtransform( ...
                container, ...
                "DisplayName","Transform",...
                "Tag","Transform");

            % Get the gizmo appearance properties
            gizmoParams = Graphics.GizmoProperties();

            % Return if no numerics
            if this.IsSymbolic
                warning("Unable to plot SO3 group as its transform is symbolic, drawing identity.");
                set(th,"Matrix",eye(4));
            else
                % Attempt to get numeric instance of this transform
                m = this.transform * Transform.Scale(this.scale);
                set(th,"Matrix",m);                
            end

            % Draw a triad at the location
            hTri = Graphics.DrawTriad(gizmoParams.scale);                                                        
            set(hTri,'Parent',th);
        end
        function [this] = Reset(this)
            % Reset to default transform
            this.transform = eye(4);
        end
        function [s] = WorldScale(this)
            % This scale multiplied by all the parent scales

            % [To fix after parentage]
            s = this.scale;
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
        function [this] = OnValueChanged(this)
            % This event is notified when the transform value is updated.
            notify(this,"ValueChanged");
        end
    end
    events (NotifyAccess = private)
        ValueChanged;
    end

    %% Tools (Calculations with transforms)
    methods (Static)
        % Transform kinematic properties by a transform
        function [twist,twistRate] = TransformMotion(T,twist0,twistRate0)
            % This function transforms motion in a child frame, into motion
            % in a parent frame.
            
            % This function assumes the transformation is "static" i.e two
            % point on the same body that are rigidly connected.

            R = Transform.ToRotation(T);
            p = Transform.ToPosition(T);

            % Match numeric or sym
            twist = twist0;
            twistRate = twistRate0;

            % Calcukate the velocites
            twist(1:3,1) = twist0(1:3,1);
            twist(4:6,1) = twist0(4:6,1) + R*(cross(twist0(1:3),p));
            % Calculate the accelerations
            twistRate(1:3,1) = zeros(3,1);
            twistRate(4:6,1) = twistRate0(4:6,1) + R*(cross(twist0(1:3,1),cross(twist0(1:3,1),p)));
        end

        % Transform a point
        function [p] = TransformPoint(T,p0)
            % Note that the returned position is affected by scale.
            % Transforms a position from local space to parent space.

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            assert(IsColumn(p0,3),"Expecting a [3x1] position to map to parent space.");
            p = T*[p0;1];
            p = p(1:3,1);
        end
        % Inverse-transform a point
        function [p] = InverseTransformPoint(T,p0)
            % Note that the returned position is affected by scale.
            % Transforms a position from parent space to local space.

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            assert(IsColumn(p0,3),"Expecting a [3x1] position to map to local space.");
            p = T'*[p0;1];
            p = p(1:3,1);
        end
        % Transform a direction vector
        function [v] = TransformDirection(T,v0)
            % This operation is not affected by scale or position of the
            % transform. 
            % - Transforms a direction: local space -> parent space.

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            assert(IsColumn(v0,3),"Expecting a [3x1] direction vector to map to parent space.");
            Td = Transform.DirectionOnly(T);
            v = Td*[v0;0];
            v = v(1:3,1);
        end
        % Inverse-transform a direction vector
        function [v] = InverseTransformDirection(T,v0)
            % This operation is not affected by scale or position of the
            % transform.
            % - Transforms a direction: parent space -> local space.

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            assert(IsColumn(v0,3),"Expecting a [3x1] direction vector to map to local space.");
            Td = Transform.DirectionOnly(T)';
            v = Td*[v0;0];
            v = v(1:3,1);
        end
    end
    methods (Static, Access = private)
        function [T] = DirectionOnly(T)
            T(4,4) = 0;
        end
        function [T] = TranslationOnly(T)
            T(4,4) = 0;
        end
    end

    %% Conventions
    methods (Static)
        % Adjoint matrix of a transform
        function [adT] = Adjoint(tMatrix)
            % Sanity check
            assert(IsSquare(tMatrix,4),"Expecting a [4x4] transformation matrix");
            p = Transform.ToPosition(tMatrix);
            R = Transform.ToRotation(tMatrix);
            % Create the adjoint representation of the transformation matrix
            %adT = [ R, zeros(3); Skew(p)*R, R]; % [6x6]
            adT = [ R, Skew(p)*R; zeros(3), R]; % [6x6]
        end
        % Scale a transform
        function [tMatrix] = Scale(s)
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
            tMatrix = [S(1),0,0,0; 0,S(2),0,0; 0,0,S(3),0; 0,0,0,1];
        end
        % Normalise a transform
        function [Tn] = Normalise(T)
            % This function normalises the transform representation.
            R = Transform.ToRotation(T);
            p = Transform.ToPosition(T);
            j = R(1:3,2);
            k = R(1:3,3);
            % Recalculate perpendicular assets
            i = cross(j, k);         % N = O x A
            j = cross(k, i);         % O = A x N
            % Reassign rotation
            Tn = Transform.FromPose(p,[Unit(i),Unit(j),Unit(k)]);
        end
    end

    %% High-level Tools
    methods (Static)
        function [Tsym] = Symbolic()
            % Generate a symbolic version of the transform.
            s = sym("q%d",[6,1],"real");
            Tsym = Transform.FromScalars(s(1),s(2),s(3),s(4),s(5),s(6));
        end
        function [tMatrix] = MDH(d,theta,a,alpha)
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
            tMatrix = [   ct,     -st,    0,      a;
                st*ca,   ct*ca,   -sa, -sa*d;
                st*sa,   ct*sa,    ca,  ca*d;
                0,       0,     0,     1];
        end
        function [tMatrix] = DH(theta,a,d,alpha)
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
            tMatrix = [ct, -st*ca,   st*sa, a*ct;
                st,  ct*ca,  -ct*sa, a*st;
                0,     sa,      ca,    d;
                0,      0,       0,    1];
        end
    end

    %% Creation Tools (Static)
    methods (Static)
        function [T] = AxisAngle(axis,angle)
            % Axis + angle -> Transform,  simply assume unit axes for now
            assert(IsColumn(axis,3) && norm(axis) == 1,"Axis must be a unit vector.");
            assert(isscalar(angle),"Angle must be scalar.");
            T = Transform.FromEulers(axis*angle);
        end
        function [T] = FromAxisDisplacements(x,y,z,rx,ry,rz)
            % Axis displacements -> Transform
            % Sanity check
            assert(numel(x) == 1,"Expecting a scalar x displacement.");
            assert(numel(y) == 1,"Expecting a scalar y displacement.");
            assert(numel(z) == 1,"Expecting a scalar z displacement.");
            assert(numel(rx) == 1,"Expecting a scalar x rotation.");
            assert(numel(ry) == 1,"Expecting a scalar y rotation.");
            assert(numel(rz) == 1,"Expecting a scalar z rotation.");
            % Define parameters
            Pxyz = [x;y;z];                 % Translate about x,y,z
            
            Tr = Transform.FromEulers(rx,ry,rz);
            Tp = Transform.FromPosition(Pxyz);
            % Aggregate
            T = Tp*Tr;
        end
        function [eulers] = ToEulers(T)
            % Rotation matrix -> Eulers
            % Rotation of the fixed to the global pose

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix.");

            % Extract the rotation
            R = Transform.ToRotation(T);
            sy = sqrt(R(1,1) * R(1,1) +  R(2,1) * R(2,1));
            singular = sy < 1e-6;
            if ~singular || isSym
                x = atan2(R(2,1), R(1,1));
                y = atan2(-R(3,1), sy);
                z = atan2(R(3,2) , R(3,3));                
            else
                x = 0;
                y = atan2(-R(3,1), sy);
                z = atan2(-R(2,3), R(2,2));
            end
            % Collate
            eulers = [x; y; z];
        end
        function [T] = FromEulers(phi,theta,psi)
            % Eulers -> rotation matrix
            % INPUTS
            % phi   - Roll angle (x-axis)
            % theta - Pitch angle (y-axis)
            % psi   - Yaw angle (z-axis)
            % OUTPUT
            % Reta - Combined rotations about the zyx axes

            % Input sanity check
            if nargin == 3
                eta = [phi;theta;psi];
            elseif nargin == 1
                eta = phi; % phi = [phi,theta,psi]
            elseif nargin == 0
                eta = [sym('phi_t','real');sym('theta_t','real');sym('psi_t','real')];
            else
                error('Please provide either a vector of Euler angle or a series of Euler angles.');
            end

            % Check the inputs
            assert(numel(eta) == 3,'Euler angles must be provided by a vector [3x1].');
            assert(isa(eta,'sym') || isnumeric(eta),'Euler angles must be a symbolic or numeric value.');

            % Get the progressive rotation matrices
            Rx = R_x(eta(1));
            Ry = R_y(eta(2));
            Rz = R_z(eta(3));
            % Compute the rotations in Z-Y-X order
            T = Transform.FromRotation(Rz*Ry*Rx);
        end
        function [p,R] = ToPose(T)
            % Transform -> Position & Rotation
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix.");
            p = Transform.ToPosition(T);
            R = Transform.ToRotation(T);
        end
        % Position & Rotation -> Transform
        function [T] = FromPose(p,R)
            assert(IsColumn(p,3),"Expecting a 3D Cartesian displacement [3x1].");
            assert(IsSquare(R,3),"Expecting a rotation matrix [3x3].");
            % Define the homogenous transformation matrix
            T = [R,p;0,0,0,1];
        end
        function [R] = ToRotation(T)
            % Transform -> Rotation (SO(2) or SO(3))
            d = size(T);
            assert(d(1) == d(2), 'RTB:t2r:badarg', 'matrix must be square');
            assert(any(d(1) == [3 4]), 'RTB:t2r:badarg', 'argument is not a homogeneous transform (sequence)');

            n = d(1);     % works for SE(2) or SE(3)

            if numel(d) == 2
                % single matrix case
                R = T(1:n-1,1:n-1);
            else
                %  matrix sequence case
                R = zeros(3,3,d(3));
                for i=1:d(3)
                    R(:,:,i) = T(1:n-1,1:n-1,i);
                end
            end
        end
        function [T] = FromRotation(R)
            % Convert a rotation into a transformation matrix.

            % Rotation -> Transform
            d = size(R);
            assert(d(1) == d(2), 'RTB:r2t:badarg', 'matrix must be square');
            assert(any(d(1) == [2 3]), 'RTB:r2t:badarg', 'argument is not a rotation matrix (sequence)');

            Z = zeros(d(1),1);
            B = [Z' 1];

            if numel(d) == 2
                % single matrix case
                T = [R Z; B];
            else
                %  matrix sequence case
                T = zeros(4,4,d(3));
                for i=1:d(3)
                    T(:,:,i) = [R(:,:,i) Z; B];
                end
            end
        end
        function [p] = ToPosition(T)
            % Transform -> Position
            x = size(T);
            assert(IsSquare(T),'matrix must be square');
            assert(any(x(1) == [3 4]),'argument is not a homogeneous transform (sequence)');

            if x(2) == 3                % 2D transformation
                % single matrix case
                p = T(1:2,3);
            elseif x(2) == 4
                %  matrix sequence case
                p = T(1:3,4);
            else
                error('The matrix has incompatable dimensions.');
            end
        end
        function [T] = FromPosition(p)
            % Position -> Transform
            x = size(p);
            assert(x(2) == 1, 'The input must be a column vector.');
            assert(any(x(1) == [2 3]),'Expecting a column vector of length 2 or 3.');

            % Assign the translation vector
            if x(1) == 2        % 2D transformation
                T = eye(3);
                T(1:2,3) = p;   % Assign 2D vector
            elseif x(1) == 3    % 3D transformation
                T = eye(4);
                T(1:3,4) = p;   % Assign 3D vector
            else
                error('The vector has incompatable dimensions.');
            end
        end
    end
end