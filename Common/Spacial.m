classdef Spacial
    % Consolidation class for all spacial motion tools and
    % supporting information.

    % Notation
    % v = [w_x,w_y,w_z,v_x,v_y,v_z]^T
    % a = [dw_x,dw_y,dw_z,a_x,a_y,a_z]^T
    % f = [M_x,M_y,M_z,f_x,f_y,f_z]^T

    properties (Constant)
        Dimensions = 6;
    end

    %% General spacial math tools
    methods (Static)
        % Spacial force vector skew matrix
        function [vf_cross] = ForceCross( vf )
            % This function computes the spacial cross product matrix for a spacial
            % FORCE vector (f). The use of this is c = Skew(v)*f

            % Sanity check
            assert(IsColumn(vf,Spacial.Dimensions),"Expecting a spacial force-vector [6x1].")

            % Construct the spacial cross matrix
            vf_cross = -Spacial.MotionCross(vf)';
        end
        % Spacial motion vector skew matrix
        function [vm_cross] = MotionCross(vm)
            % This function computes the spacial cross product matrix for a spacial
            % MOTION vector (v,a). The use of this is c = Skew(a)*b.

            % Sanity check
            assert(IsColumn(vm,Spacial.Dimensions),"Expecting a spacial motion-vector [6x1].")

            % Construct the spacial cross matrix
            vm_cross = [
                Skew(vm(1:3,1)),zeros(3);
                Skew(vm(4:6,1)),Skew(vm(1:3,1))];
        end
        % Spacial inertia matrix
        function [rbi] = Inertia( m, com, I )
            % mcI  spatial rigid-body inertia from mass, CoM and rotational inertia.
            % mcI(m,c,I) calculates the spatial inertia matrix of a rigid body from its
            % mass, centre of mass (3D vector) and rotational inertia (3x3 matrix)
            % about its centre of mass.

            % Sanity check
            assert(isscalar(m),"Expecting a scalar mass");
            assert(IsColumn(com,3),"Expecting a Cartesian center of mass [3x1].");
            assert(IsSquare(I,3),"Expecting a [3x3] inertia matrix.");

            % COM cross product matrix
            C = Skew(com);

            rbi = [ I + m*C*C', m*C; m*C', m*eye(3) ];
        end
    end

    %% Spacial [6x6] Coordinate transformations
    methods (Static)
        function [T] = X2T(X)
            % This function extracts a homogenonous transform from the
            % spacial transform.

            R = X(1:3,1:3);
            p = DeSkew(X(4:6,1:3));
            % Create the transform
            T = Transform.FromPose(p,R);
        end
        function [X] = T2X(T)
            % This function generates a spacial transform from a standard
            % homogenous transform.

            % Extract the pose
            p = Transform.ToPosition(T);
            R = Transform.ToRotation(T);
            % Create the Spacial Transform
            [X] = Spacial.Transform(p,R);
        end
        function [X] = Transform(p,R)
            % Calculate a combined translation and rotation spacial
            % transform.
            Xt = Spacial.Translation(p);
            Xr = Spacial.RotationMatrix(R);
            X = Xr*Xt;
        end
        function [Xtrans] = Translation(p)
            % This function creates a spacial coordinate transform, origin
            % to new origin. This represents the transformation of frame B
            % of frame A by p.

            Xtrans = eye(6);
            if isa(p,"sym")
                Xtrans = sym(Xtrans);
            end
            Xtrans(4:6,1:3) = -Skew(p);
        end
        function [Xr] = RotationMatrix(R)
            assert(IsSquare(R,3),"Expecting a valid rotation matrix.");
            Xr = [R,zeros(3);zeros(3),R];
        end
        function [Xzyx] = RotationAngles(angles)
            % This function generates a rotation matrix that combines the 
            % rotations about the z-axis, y-axis and x-axis, in that order.

            assert(numel(angles) == 3,"Expecting an array of euler angles.");
            % Generate the matrix
            Xzyx = ZRotation(angles(3))*YRotation(angles(2))*XRotation(angles(1));
        end
        function [Xx] = XRotation( theta )
            % Spatial coordinate transform (X-axis rotation).
            % This calculates the coordinate transform matrix from A to B
            % coordinates for spatial motion vectors, where coordinate frame B is
            % rotated by an angle theta (radians) relative to frame A about their
            % common X axis.

            %         c = cos(theta);
            %         s = sin(theta);
            %
            %         Xa = [ 1  0  0  0  0  0 ;
            %               0  c  s  0  0  0 ;
            %               0 -s  c  0  0  0 ;
            %               0  0  0  1  0  0 ;
            %               0  0  0  0  c  s ;
            %               0  0  0  0 -s  c
            %             ];
            Xx = [R_x(theta),zeros(3);zeros(3),R_x(theta)];
        end
        function [Xy] = YRotation( theta )
            % Spatial coordinate transform (Y-axis rotation).
            % This calculates the coordinate transform matrix from A to B
            % coordinates for spatial motion vectors, where coordinate frame B is
            % rotated by an angle theta (radians) relative to frame A about their
            % common Y axis.
            Xy = [R_y(theta),zeros(3);zeros(3),R_y(theta)];
        end
        function [Xz] = ZRotation( theta )
            % Spatial coordinate transform (z-axis rotation).
            % This calculates the coordinate transform matrix from A to B
            % coordinates for spatial motion vectors, where coordinate frame B is
            % rotated by an angle theta (radians) relative to frame A about their
            % common Z axis.
            Xz = [R_z(theta),zeros(3);zeros(3),R_z(theta)];
        end
    end
end