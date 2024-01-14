classdef PhysicsExtensions
    %PHYSICSEXTENSIONS Summary of this class goes here
    %   Detailed explanation goes here

    %% Quaternion Extensions
    methods (Static)
        function [T] = QuaternionTransform(p,q)
            % This function creates a quaternion transform
            Rq = PhysicsExtensions.QuaternionToRotation(q);
            % Create the equivalent transform
            T = PhysicsExtensions.PoseToTransform(p,Rq);
        end
        function [q] = TransformToQuaternion(T)
            R = TransformToRotation(T);
            q = RotationToQuaternion(R);
        end
        function [R] = QuaternionToRotation(q)
            % This function computes the rotation matrix of the quaternion
            % variables describing the 3D rotations of 3D body.

            % Input sanity check
            if nargin < 1
                q = sym('q%d',[4,1],'real');
            end
            assert(numel(q) == 4,'The quaternion must be of format [4x1]');
            
            isSym = false;
            if isa(q,"sym")
                isSym = true;
            else
                q = PhysicsExtensions.qUnit(q);  % Normalise the quaternion
            end

            % Output container
            R = zeros(3);
            if isSym
                R = sym(R);
            end

            % Define the quaternion rotation matrix
            R(1,1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;    
            R(1,2) = 2*(q(2)*q(3) - q(1)*q(4));
            R(1,3) = 2*(q(1)*q(3) + q(2)*q(4));
            R(2,1) = 2*(q(2)*q(3) + q(1)*q(4));
            R(2,2) = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
            R(2,3) = 2*(q(3)*q(4) - q(1)*q(2));
            R(3,1) = 2*(q(2)*q(4) - q(1)*q(3));
            R(3,2) = 2*(q(1)*q(2) + q(3)*q(4));
            R(3,3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;

            % Reduce where possible
%             R = SymTools.Reduce(R);
        end
        function [q] = RotationToQuaternion(R)
            % This function is designed to convert from a rotation matrix
            % to an equivalent quaternion. This function is also parallel
            % to "rotm2quat.m".

            % Sanity check
            assert(IsRotationMatrix(R),"Expecting a valid rotation matrix [3x3].");

            % The trace
            tr = R(1,1) + R(2,2) + R(3,3);

            % Container
            q = zeros(4,1);
            if isa(R,"sym")
                q = sym(q);
                % Assume condition #1
                S = sqrt(tr + 1.0) * 2; 
                q(1) = 0.25 * S;
                q(2) = (R(3,2) - R(2,3)) / S;
                q(3) = (R(1,3) - R(3,1)) / S;
                q(4) = (R(2,1) - R(1,2)) / S;
                % Reduce if possible
                q = SymTools.Reduce(q);
                return
            end

            % OUTPUT Matches rotm2quat
            if tr > 0 
                S = sqrt(tr + 1.0) * 2; 
                q(1) = 0.25 * S;
                q(2) = (R(3,2) - R(2,3)) / S;
                q(3) = (R(1,3) - R(3,1)) / S;
                q(4) = (R(2,1) - R(1,2)) / S;
                % is valid
            elseif ((R(1,1) > R(2,2)) && (R(1,1) > R(3,3)))
                S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*q(2)
                q(1) = (R(3,2) - R(2,3)) / S;
                q(2) = 0.25 * S;
                q(3) = (R(1,2) + R(2,1)) / S;
                q(4) = (R(1,3) + R(3,1)) / S;
                % is valid
            elseif (R(2,2) > R(3,3))
                S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S=4*q(3)
                q(1) = (R(1,3) - R(3,1)) / S;
                q(2) = (R(1,2) + R(2,1)) / S;
                q(3) = 0.25 * S;
                q(4) = (R(2,3) + R(3,2)) / S;
                % to validate
            else
                S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S=4*q(4)
                q(1) = (R(2,1) - R(1,2)) / S;
                q(2) = (R(1,3) + R(3,1)) / S;
                q(3) = (R(2,3) + R(3,2)) / S;
                q(4) = 0.25 * S;
                % to validate
            end
            % Reduce where possible
%             q = SymTools.Reduce(q);
        end
        function [phi,theta,psi] = QuaternionToEulers(q)
            % Sanity check
            assert(numel(q) == 4,"Expecting quaternion vector [4x1].");

            % Compute the euler rotation from a unit quaternion
            phi = atan2(2*(q(1)*q(2) + q(3)*q(4)),(1 - 2*(q(2)^2 + q(3)^2)));
            theta = asin(2*(q(1)*q(3) - q(4)*q(2)));
            psi = atan2(2*(q(1)*q(4) + q(2)*q(3)),(1 - 2*(q(3)^2 + q(4)^2)));
        end
        function [q] = EulersToQuaternion(phi,theta,psi)
            % Generate a quaternion of the equivalent euler angles.

            % Abbreviations for the various angular functions
            cy = cos(psi * 0.5);
            sy = sin(psi * 0.5);
            cp = cos(theta * 0.5);
            sp = sin(theta * 0.5);
            cr = cos(phi * 0.5);
            sr = sin(phi * 0.5);

            q = zeros(4,1);
            q(1) = cr * cp * cy + sr * sp * sy;
            q(2) = sr * cp * cy - cr * sp * sy;
            q(3) = cr * sp * cy + sr * cp * sy;
            q(4) = cr * cp * sy - sr * sp * cy;
        end
        % Operations
        function [dq] = qDifferential(q0,omega)
            % Compute the quaternion differential

            assert(numel(q0)==4,"Expecting a valid quaternion [4x1].");
            assert(IsColumn(omega,3),"Expecting a body axis rate [3x1].");

            % Rewritten to allow multiplication by omega_b directly (of [4x3])
            Jq = 0.5*[-q0(2), -q0(3), -q0(4);
                q0(1), -q0(4),  q0(3);
                q0(4),  q0(1), -q0(2);
                -q0(3), -q0(2),  q0(1)];

            dq = Jq*omega;
        end
        function [qv] = qMultiply(q,v)
            % Calculate the product of two quaternions
            % Associated block:
            % "Quaternion Multiplication"
            % Multiply the quaternion elements

            assert(size(q,1) == 4 && size(v,1) == 4,...
                'Both quaternion must be provided as 4x1 column vectors')
            % Quaternion projection matrix
            qv = [v(1), -v(2), -v(3), -v(4);
                v(2),  v(1), -v(4),  v(3);
                v(3),  v(4),  v(1), -v(2);
                v(4), -v(3),  v(2),  v(1)]*q; % Confirmed with matlab website
        end
        function [q_hat] = qUnit(q)
            % This function normalises the quaternion
            q_hat = q/sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
        end
    end

    %% Euler Extensions
    methods (Static)
        function [T] = EulerTransform(x,y,z,rx,ry,rz)
            % Axis displacements -> Transform
            % Sanity check
            assert(isscalar(x),"Expecting a scalar x displacement.");
            assert(isscalar(y),"Expecting a scalar y displacement.");
            assert(isscalar(z),"Expecting a scalar z displacement.");
            assert(isscalar(rx),"Expecting a scalar x rotation.");
            assert(isscalar(ry),"Expecting a scalar y rotation.");
            assert(isscalar(rz),"Expecting a scalar z rotation.");
            % Define parameters
            p = [x;y;z];                 % Translate about x,y,z
            % The euler rotation
            Re = EulersToRotation(rx,ry,rz);
            % Compound transform
            T = PoseToTransform(p,Re);
        end
        function [phi,theta,psi] = TransformToEulers(T)
            % Rotation matrix -> Eulers
            % Rotation of the fixed to the global pose

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix.");

            % Extract the rotation
            R = TransformToRotation(T);
            [phi,theta,psi] = RotationToEulers(R);
        end
        function [T] = EulersToTransform(phi,theta,psi)
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

            % Euler rotation
            Re = EulersToRotation(eta(1),eta(2),eta(3));
            % The transform
            T = RotationToTransform(Re);
        end
        function [phi,theta,psi] = RotationToEulers(R)
            % Calculate the euler rotations from a given rotation matrix.

            % Sanity check
            assert(IsRotationMatrix(R),"Expecting a valid rotation matrix.");
 
            sy = sqrt(R(1,1) * R(1,1) +  R(2,1) * R(2,1));
            singular = sy < 1e-6;
            if ~singular
                phi = atan2(R(2,1),R(1,1));
                theta = atan2(-R(3,1),sy);
                psi = atan2(R(3,2),R(3,3));                
            else
                phi = 0;
                theta = atan2(-R(3,1),sy);
                psi = atan2(-R(2,3),R(2,2));
            end
        end
        function [R] = EulersToRotation(phi,theta,psi)
            % Calculate the rotation matrix from euler representation
            R = R_x(phi)*R_y(theta)*R_z(psi);
        end
    end

    %% Transform Extensions
    methods (Static)
        % Operations
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
        function [p] = TransformPoint(T,p0)
            % Note that the returned position is affected by scale.
            % Transforms a position from local space to parent space.

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            assert(IsColumn(p0,3),"Expecting a [3x1] position to map to parent space.");
            p = T*[p0;1];
            p = p(1:3,1);
        end
        function [p] = InverseTransformPoint(T,p0)
            % Note that the returned position is affected by scale.
            % Transforms a position from parent space to local space.

            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            assert(IsColumn(p0,3),"Expecting a [3x1] position to map to local space.");
            p = T'*[p0;1];
            p = p(1:3,1);
        end
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
        % Matrix operations
        function [T] = Normalise(T)
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
        function [T] = DirectionOnly(T)
            T(4,4) = 0;
        end
        function [T] = TranslationOnly(T)
            T(4,4) = 0;
        end
        % (Advanced) Matrix creation
        function [M] = Adjoint(T)
            % Adjoint matrix of a transform
            
            % Sanity check
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix");
            p = Transform.ToPosition(T);
            R = Transform.ToRotation(T);
            % Create the adjoint representation of the transformation matrix
            %adT = [ R, zeros(3); Skew(p)*R, R]; % [6x6]
            M = [ R, Skew(p)*R; zeros(3), R]; % [6x6]
        end        
        function [T] = MDHTransform(d,theta,a,alpha)
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
            T = [   ct,     -st,    0,      a;
                st*ca,   ct*ca,   -sa, -sa*d;
                st*sa,   ct*sa,    ca,  ca*d;
                0,       0,     0,     1];
        end
        function [T] = DHTransform(theta,a,d,alpha)
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
            T = [ct, -st*ca,   st*sa, a*ct;
                st,  ct*ca,  -ct*sa, a*st;
                0,     sa,      ca,    d;
                0,      0,       0,    1];
        end
        function [T] = ScaleTransform(s)
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
            T = [S(1),0,0,0; 0,S(2),0,0; 0,0,S(3),0; 0,0,0,1];
        end
        function [T] = SymbolicTransform()
            % Generate a symbolic version of the transform.
            s = sym("q%d",[6,1],"real");
            T = Transform.FromScalars(s(1),s(2),s(3),s(4),s(5),s(6));
        end
        % (Standard) Matrix creation
        function [p,R] = TransformToPose(T)
            % Transform -> Position & Rotation
            assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix.");
            p = Transform.ToPosition(T);
            R = Transform.ToRotation(T);
        end
        function [T] = PoseToTransform(p,R)
            assert(IsColumn(p,3),"Expecting a 3D Cartesian displacement [3x1].");
            assert(IsSquare(R,3),"Expecting a rotation matrix [3x3].");
            % Define the homogenous transformation matrix
            T = [R,p;0,0,0,1];
        end
        function [R] = TransformToRotation(T)
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
        function [T] = RotationToTransform(R)
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
        function [p] = TransformToPosition(T)
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
        function [T] = PositionToTransform(p)
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

