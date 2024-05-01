classdef FeatherstoneDynamics < DynamicsSolver
    % This class implements the Featherstone method for resolving
    % the dynamic properties of a physics simulation.

    properties (Constant)
        Name = "Featherstone Dynamic solver.";
    end
    % Interface
    methods
        function [this] = Compute(this,bodies)
            % Compute the motion of the provided bodies utilitising the
            % given approach.

            % Resolve velocities
            this.ResolveVelocities(bodies);
            % Resolve velocities
            this.ResolveAccelerations(bodies);
            % Resolve velocities
            this.ResolveForces(bodies);
        end
    end

    % Internal
    methods (Access = protected)
        function [this] = ResolveVelocities(this,transforms)
            % This function computes the velocities of all the provided
            % transforms.

            for i = 1:numel(transforms)
                transform_i = transforms(i);

                
                if isempty(transform_i.Entity.Joints)
                    continue;
                end
            end
        end
        function[this] = ResolveAccelerations(this,transforms)
            % This function computes the accelerations of all the provided
            % transforms.

            for i = 1:numel(transforms)
                transform_i = transforms(i);

                

            end
        end
        function [this] = ResolveForces(this,transforms)
            % This function computes the forces of all the provided
            % transforms.

            for i = 1:numel(transforms)
                transform_i = transforms(i);

                

            end
        end
    end

    methods (Static)    
        % Spacial mathmatics
        function [vf_cross] = ForceCross(vf)
            % This function computes the spacial cross product matrix for a spacial
            % FORCE vector (f). The use of this is c = Skew(v)*f

            % Sanity check
            assert(IsColumn(vf,6),"Expecting a spacial force-vector [6x1].")

            % Construct the spacial cross matrix
            vf_cross = -this.MotionCross(vf)';
        end
        function [vm_cross] = MotionCross(vm)
            % This function computes the spacial cross product matrix for a spacial
            % MOTION vector (v,a). The use of this is c = Skew(a)*b.

            % Sanity check
            assert(IsColumn(vm,6),"Expecting a spacial motion-vector [6x1].")

            vSkew = Skew(vm(1:3,1));
            wSkew = Skew(vm(4:6,1));
            % Construct the spacial cross matrix
            vm_cross = [vSkew,zeros(3);wSkew,vSkew];
        end
        function [rbi] = Inertia(m,com,I)
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
            % Construct the spacial mass matrix
            rbi = [ I + m*C*C', m*C; m*C', m*eye(3) ];
        end
        % Spacial matrix construction
        function [X] = FromElements(x,y,z,xa,ya,za)
            % Calculate a combined translation and rotation spacial
            % transform.
            
            % Sanity check
            assert(nargin == 6,"Expecting 6 input arguments");

            % Multiply the translation and rotation
            X = this.FromPositionRotation([x;y;z],R_xyz(xa,ya,za));
        end
        function [X] = FromSO3(so3)
            % Calculate a combined translation and rotation spacial
            % transform.
            
            % Sanity check
            assert(isa(so3,"SO3"),"Expecting a valid SO3.");

            % Multiply the translation and rotation
            X = this.FromPositionRotation(so3.Position,so3.Rotation);
        end
        function [X] = FromPositionRotation(p,R)
            % From the positions and rotation.

            Xt = this.Translation(p);
            Xr = this.Rotation(R);
            X = Xr*Xt;
        end
        function [Xt] = Translation(p)
            % This function creates a spacial coordinate transform, origin
            % to new origin. This represents the transformation of frame B
            % of frame A by p.

            % Format as the 6D matrix
            Xt = [eye(3),Skew(p);zeros(3),eye(3)];
        end
        function [Xr] = Rotation(rotationMatrix)
            % Create the spacial rotation matrix.

            % Format as the 6D matrix
            Xr = [rotationMatrix,zeros(3);zeros(3),rotationMatrix];
        end
    end
end