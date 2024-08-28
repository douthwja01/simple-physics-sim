classdef PhysicsExtensions
    %PHYSICSEXTENSIONS Summary of this class goes here
    %   Detailed explanation goes here

    %% Rotation Extensions
    methods (Static)
        % Quaternion Transformations
        function [T] = QuaternionTransform(p,q)
            assert(isa(q,"Quaternion"),"Expecting a valid quaternion.");
            % This function creates a quaternion transform
            Rq = q.ToRotation();
            % Create the equivalent transform
            T = PhysicsExtensions.PoseToTransform(p,Rq);
        end
        function [q] = TransformToQuaternion(T)
            R = TransformToRotation(T);
            q = RotationToQuaternion(R);
        end
        % Euler transformation
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
        % Euler rotations
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
        % (Standard) Matrix creation
%         function [p,R] = TransformToPose(T)
%             % Transform -> Position & Rotation
%             assert(IsSquare(T,4),"Expecting a [4x4] transformation matrix.");
%             p = Transform.ToPosition(T);
%             R = Transform.ToRotation(T);
%         end
%         function [T] = PoseToTransform(p,R)
%             assert(IsColumn(p,3),"Expecting a 3D Cartesian displacement [3x1].");
%             assert(IsSquare(R,3),"Expecting a rotation matrix [3x3].");
%             % Define the homogenous transformation matrix
%             T = [R,p;0,0,0,1];
%         end
%         function [R] = TransformToRotation(T)
%             % Transform -> Rotation (SO(2) or SO(3))
%             d = size(T);
%             assert(d(1) == d(2), 'RTB:t2r:badarg', 'matrix must be square');
%             assert(any(d(1) == [3 4]), 'RTB:t2r:badarg', 'argument is not a homogeneous transform (sequence)');
% 
%             n = d(1);     % works for SE(2) or SE(3)
% 
%             if numel(d) == 2
%                 % single matrix case
%                 R = T(1:n-1,1:n-1);
%             else
%                 %  matrix sequence case
%                 R = zeros(3,3,d(3));
%                 for i=1:d(3)
%                     R(:,:,i) = T(1:n-1,1:n-1,i);
%                 end
%             end
%         end
%         function [T] = RotationToTransform(R)
%             % Convert a rotation into a transformation matrix.
% 
%             % Rotation -> Transform
%             d = size(R);
%             assert(d(1) == d(2), 'RTB:r2t:badarg', 'matrix must be square');
%             assert(any(d(1) == [2 3]), 'RTB:r2t:badarg', 'argument is not a rotation matrix (sequence)');
% 
%             Z = zeros(d(1),1);
%             B = [Z' 1];
% 
%             if numel(d) == 2
%                 % single matrix case
%                 T = [R Z; B];
%             else
%                 %  matrix sequence case
%                 T = zeros(4,4,d(3));
%                 for i=1:d(3)
%                     T(:,:,i) = [R(:,:,i) Z; B];
%                 end
%             end
%         end
%         function [p] = TransformToPosition(T)
%             % Transform -> Position
%             x = size(T);
%             assert(IsSquare(T),'matrix must be square');
%             assert(any(x(1) == [3 4]),'argument is not a homogeneous transform (sequence)');
% 
%             if x(2) == 3                % 2D transformation
%                 % single matrix case
%                 p = T(1:2,3);
%             elseif x(2) == 4
%                 %  matrix sequence case
%                 p = T(1:3,4);
%             else
%                 error('The matrix has incompatable dimensions.');
%             end
%         end
%         function [T] = PositionToTransform(p)
%             % Position -> Transform
%             x = size(p);
%             assert(x(2) == 1, 'The input must be a column vector.');
%             assert(any(x(1) == [2 3]),'Expecting a column vector of length 2 or 3.');
% 
%             % Assign the translation vector
%             if x(1) == 2        % 2D transformation
%                 T = eye(3);
%                 T(1:2,3) = p;   % Assign 2D vector
%             elseif x(1) == 3    % 3D transformation
%                 T = eye(4);
%                 T(1:3,4) = p;   % Assign 3D vector
%             else
%                 error('The vector has incompatable dimensions.');
%             end
%         end
    end
end

