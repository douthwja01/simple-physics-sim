classdef FeatherstoneDynamics < DynamicsSolver
    % This class implements the Featherstone method for resolving
    % the dynamic properties of a physics simulation.

    properties (Constant)
        Name = "Featherstone Dynamic solver.";
    end
    properties
        Map = double.empty;
        Xup = cell.empty;   % Spacial transforms
        S = cell.empty;     % Spacial joint subspaces
        v = cell.empty;     % Spacial velocities
        c = cell.empty;     % Spacial Coriolis terms
        IA = cell.empty;    % Spacial articulation inertia
        pA = cell.empty;    % Spacial articulation force
        U = cell.empty;
        d = cell.empty;
        u = cell.empty;
        g = [0;0;0;0;0;-9.81];
    end

    % Interface
    methods
        function [this] = Compute(this,bodies)
            % Compute the motion of the provided bodies utilitising the
            % given approach.

            % Reinitialise map
            nBodies = numel(bodies);
            this.Map = zeros(nBodies,2);

            for i = 1:numel(bodies)
                body_i = bodies(i);
                this.Map(i,1) = body_i.Uid;
                if body_i.NumberOfParents > 0
                    this.Map(i,2) = body_i.Parent.Uid;
                end
            end

            % Resolve velocities
            this.ResolveVelocities(bodies);
            % Resolve forces
            this.ResolveForces(bodies);
            % Resolve Accelerations
            this.ResolveAccelerations(bodies);
        end
    end

    % Internal
    methods (Access = protected)
        function [this] = ResolveVelocities(this,transforms)
            % This function computes the velocities of all the provided
            % transforms.

            % Initialise articulation containers
            n = numel(transforms);
            this.Xup = cell(n,1);
            this.S = cell(n,1);
            this.v = cell(n,1);
            this.c = cell(n,1);

            for i = 1:n
                transform_i = transforms(i);
                fprintf("Calculating velocities for %s joints.\n",transform_i.Entity.Name);
                
                % Get the joint
                XLink = FeatherstoneDynamics.FromSO3(transform_i.Local);    % Get the local transformation
                joint = transform_i.Entity.Joints;

                if isempty(joint)
                    % Assume not articulated
                    Xup_i = XLink;      
                    S_i = zeros(6,1);                                       % No connection
                    v_i = [transform_i.AngularVelocity;transform_i.Velocity];
                    c_i = FeatherstoneDynamics.MotionCross(v_i);
                else
                    % Get the spacial joint data
                    [XJ,S_i,vJ] = FeatherstoneDynamics.GetJointData(joint);
                    % Transformation through joint
                    Xup_i = XJ * XLink;
                    % Compute resulting velocity terms
                    if transform_i.NumberOfParents == 0
                        v_i = zeros(6,1);
                        c_i = zeros(6,1);
                    else
                        % Parent velocity terms
                        v_p = zeros(6,1);
                        v_p(1:3,1) = transform_i.Parent.AngularVelocity;
                        v_p(4:6,1) = transform_i.Parent.Velocity;
                        v_i = Xup_i*v_p + vJ;
                        c_i = FeatherstoneDynamics.MotionCross(v_i) * vJ;
                    end
                end
                % Assign to containers
                this.Xup{i} = Xup_i;
                this.S{i} = S_i;
                this.v{i} = v_i;
                this.c{i} = c_i;

                % Pass velocity data to the transform
                transform_i.AngularVelocity = v_i(1:3,1);
                transform_i.Velocity = v_i(4:6,1);
            end
        end
        function [this] = ResolveForces(this,transforms)
            % This function computes the forces of all the provided
            % transforms.

            % Container setup
            n = numel(transforms);
            this.IA = cell(n,1);
            this.pA = cell(n,1);
            this.U = cell(n,1);
            this.d = cell(n,1);
            this.u = cell(n,1);

            % Calculate the inertial (force + velocity) contributions 
            for i = 1:1:n
                transform_i = transforms(i);
                fprintf("Calculating forces for %s joints.\n",transform_i.Entity.Name);
                
                rb_i = transform_i.Entity.RigidBody;
                v_i = this.v{i};

                assert(~isempty(rb_i),"Cannot handle non-rigidbody components yet.");

                % Body properties
                IA_i = FeatherstoneDynamics.Inertia(rb_i.Mass,rb_i.CenterOfMass,rb_i.Inertia);
                % The net force and torques acting on the body
                f_i = [rb_i.NetTorque;rb_i.NetForce];                       
                % Sum the inertial-velocity and external force/torque terms
                pA_i = FeatherstoneDynamics.ForceCross(v_i) * IA_i * v_i - f_i;
                % Store properties
                this.IA{i} = IA_i;  % Always [6x6]
                this.pA{i} = pA_i;  % Always [6x1]
            end

            % Calculate the joint subspace inertial properites
            for i = n:-1:1
                transform_i = transforms(i);
                % Transform
                Xup_i = this.Xup{i};
                c_i  = this.c{i};
                IA_i = this.IA{i};
                pA_i = this.pA{i};
                S_i  = this.S{i};
                Si_trans = transpose(S_i);

                % Get the joint
                joint = transform_i.Entity.Joints;
                if isempty(joint)
                    continue;
                end

                if isa(joint,"MovableJoint")
                    % Get equivalent inputs
                    if isa(joint,"ActuatedJoint")
                        tau_i = joint.JointInput;
                    else
                        tau_i = zeros(joint.DegreesOfFreedom,1);
                    end

                    U_i = IA_i * S_i;
                    d_i = Si_trans * U_i;
                    u_i = tau_i - Si_trans*pA_i;
                else
                    U_i = double.empty;
                    d_i = double.empty;
                    u_i = double.empty;
                end

                this.U{i} = U_i;
                this.d{i} = d_i;
                this.u{i} = u_i;

                % = Parents Check (contributions to the parent inertials) =
                if transform_i.NumberOfParents == 0
                    continue;
                end

                % Get the index of the parent (who's properties we're
                % modifying)
                [flag,ind] = this.GetParentIndex(transform_i.Uid);
                if ~flag
                    % This should only be triggered if the parent found is
                    % not within the map (therefore not a body i.e the world.)
                    continue
                end

                % Calcuate the inertial and force components acting through
                % the joint subspace.
                Ia = IA_i - U_i/d_i*U_i';
                pa = pA_i + Ia*c_i + U_i * u_i/d_i;
                % Append the inertial contributions to the parent
                this.IA{ind} = this.IA{ind} + Xup_i' * Ia * Xup_i;
                this.pA{ind} = this.pA{ind} + Xup_i' * pa;
            end
        end
        function [this] = ResolveAccelerations(this,transforms)
            % This function computes the accelerations of all the provided
            % transforms.

            
            for i = 1:numel(transforms)
                transform_i = transforms(i);
                fprintf("Calculating accelerations for %s joints.\n",transform_i.Entity.Name);
                
                Xup_i = this.Xup{i};
                S_i = this.S{i};
                c_i = this.c{i};

                joint = transform_i.Entity.Joints;

                % Condition 1: No joint
                if isempty(joint)
                    this.a{i} = Xup_i * -this.g + c_i;
                    continue;
                end

                if isa(joint,"MovableJoint")

                else

                end

                
                [flag,ind] = this.GetParentIndex(transform_i.Uid);
                if ~flag 
                    a_0 = -this.g;      % No joint, accelerate with gravity
                else
                    a_0 = this.a{ind};  % Joint, accelerate with parent
                end

                % Get the acceleration of the link
                a_i = Xup_i * a_0 + c_i;
                % Get the acceleration through the joint subspace.
                qdd_i = (this.u{i} - this.U{i}'*a_i)/this.d{i};
                % The resulting acceleration of the link
                a_i = a_i + S_i*qdd_i;
                % Store the acceleration
                this.a{i} = a_i;

                % == Pass velocity data to the transform ==
                transform_i.AngularAcceleration = a_i(1:3,1);
                transform_i.Acceleration = a_i(4:6,1);
            end
        end
        function [flag,ind,pid] = GetParentIndex(this,uid)
            % This function gets the parent properties of a given transform
            % in the flattened manner.
            % flag  - A parent was found.
            % ind   - Its index in the map
            % pid   - The parent's uid

            selectionLogical = this.Map(:,1) == uid;
           
            ind = -1;
            flag = false;
            for i = 1:numel(selectionLogical)
                if selectionLogical == 1
                    ind = i;
                    flag = true;
                    break
                end
            end

            % Parent id 
            pid = this.Map(selectionLogical,2);
        end
    end
    methods (Static)
        % Spacial mathmatics
        function [Xj,S,vJ]  = GetJointData(joint)
            % This function provides a breakout from the JCalc method to
            % the joint definitions
            switch joint.Code
                case JointCode.Fixed
                    % Create the fixed joint spacial properties
                    Xj = eye(6);
                    S = zeros(6,1);
                    dq = 0;
                case JointCode.Revolute
                    % Create the revolute joint spacial properties
                    q = joint.JointPosition;
                    dq = joint.JointVelocity;
                    Xj = FeatherstoneDynamics.FromRotation(R_x(q));
                    S = [0;0;1;0;0;0];
                case JointCode.Prismatic
                    % Create the prismatic joint spacial properties
                    q = joint.JointPosition;
                    dq = joint.JointVelocity;
                    Xj = FeatherstoneDynamics.FromTranslation([0;0;q]);
                    S = [0;0;0;0;0;1];
                otherwise
                    error("Joint code %s not implemented.",joint.Code);
            end
            % The spacial joint velocity
            vJ = S*dq;
        end
        function [Xj,S]     = JointCalc( pitch, q )
            % jcalc  Calculate joint transform and motion subspace.
            % [Xj,S]=jcalc(pitch,q) calculates the joint transform and motion subspace
            % matrices for a revolute (pitch==0), prismatic (pitch==inf) or helical
            % (pitch==any other value) joint.  For revolute and helical joints, q is
            % the joint angle.  For prismatic joints, q is the linear displacement.

            if pitch == 0				% revolute joint
                Xj = this.Rotation(R_x(q));
                S = [0;0;1;0;0;0];
            elseif pitch == inf			% prismatic joint
                Xj = this.Translation([0;0;q]);
                S = [0;0;0;0;0;1];
            else					% helical joint
                %Xj = Xrotz(q) * Xtrans([0 0 q*pitch]);
                Xj = FeatherstoneDynamics.FromElements(0,0,q*pitch,q,0,0);
                S = [0;0;1;0;0;pitch];
            end
        end
        function [vf_cross] = ForceCross(vf)
            % This function computes the spacial cross product matrix for a spacial
            % FORCE vector (f). The use of this is c = Skew(v)*f

            % Sanity check
            assert(IsColumn(vf,6),"Expecting a spacial force-vector [6x1].")

            % Construct the spacial cross matrix
            vf_cross = -FeatherstoneDynamics.MotionCross(vf)';
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
            X = FeatherstoneDynamics.FromTranslationRotation([x;y;z],R_xyz(xa,ya,za));
        end
        function [X] = FromSO3(so3)
            % Calculate a combined translation and rotation spacial
            % transform.

            % Sanity check
            assert(isa(so3,"SO3"),"Expecting a valid SO3.");

            % Multiply the translation and rotation
            X = FeatherstoneDynamics.FromTranslationRotation( ...
                so3.Position, ...
                so3.Rotation.GetRotationMatrix());
        end
        function [X]  = FromTranslationRotation(p,R)
            % From the positions and rotation.

            Xt = FeatherstoneDynamics.FromTranslation(p);
            Xr = FeatherstoneDynamics.FromRotation(R);
            X = Xt*Xr;
        end
        function [Xt] = FromTranslation(p)
            % This function creates a spacial coordinate transform, origin
            % to new origin. This represents the transformation of frame B
            % of frame A by p.

            % Format as the 6D matrix
            Xt = [eye(3),Skew(p);zeros(3),eye(3)];
        end
        function [Xr] = FromRotation(rotationMatrix)
            % Create the spacial rotation matrix.

            % Format as the 6D matrix
            Xr = [rotationMatrix,zeros(3);zeros(3),rotationMatrix];
        end
    end
end