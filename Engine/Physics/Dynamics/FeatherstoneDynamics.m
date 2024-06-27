classdef FeatherstoneDynamics < DynamicsModule
    % This class implements the Featherstone method for resolving
    % the dynamic properties of a physics simulation.

    properties (Constant)
        Name = "Featherstone Dynamic solver.";
    end
    properties
        g = [0;0;0;0;0;9.81];
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
        a = cell.empty;
        qdd = cell.empty;
    end

    % We treat a non-joint like a floating-joint

    %% Internal
    methods (Access = protected)
        function [this] = TopLevelRoutine(this,bodies)
            % Compute the motion of the provided bodies utilitising the
            % given approach.

            % Reinitialise map
            n = numel(bodies);
            this.Map = zeros(n,2);
            for i = 1:n
                body_i = bodies(i);
                this.Map(i,1) = body_i.Uid;
                if body_i.Transform.NumberOfParents > 0
                    this.Map(i,2) = body_i.Transform.Parent.Uid;
                end
            end

            % Resolve velocities
            this.ComputeVelocities(bodies);
            % Resolve forces
            this.ComputeForces(bodies);
            % Resolve Accelerations
            this.ComputeAccelerations(bodies);

            % Apply properties to transforms/joints
            for i = 1:n
                body_i = bodies(i);

                joint = body_i.Entity.Joints;

                if isa(joint,"MovableJoint")
                    joint.JointAcceleration = this.qdd{i};
                end

                v_i = this.v{i};
                a_i = this.a{i};
                % == Pass velocity data to the transform ==
                body_i.AngularVelocity = v_i(1:3,1);
                body_i.LinearVelocity = v_i(4:6,1);
                body_i.AngularAcceleration = a_i(1:3,1);
                body_i.LinearAcceleration = a_i(4:6,1);
            end
        end
        function [this] = ComputeVelocities(this,bodies)
            % This function computes the velocities of all the provided
            % transforms.

            % Initialise articulation containers
            n = numel(bodies);
            this.Xup = cell(n,1);
            this.S = cell(n,1);
            this.v = cell(n,1);
            this.c = cell(n,1);

            for i = 1:n
                body_i = bodies(i);
                tf_i = body_i.Transform;
%                 fprintf("Calculating velocities for %s.\n",transform_i.Entity.Name);
                
                joint = body_i.Entity.Joints;
                % The objects with no-joint
                if isempty(joint)
                    % Velocity is the floating joint state
                    Xup_i = FeatherstoneDynamics.FromSO3(tf_i.Local); 
                    S_i = eye(6);
                    v_i = [body_i.AngularVelocity;body_i.LinearVelocity];
                    c_i = zeros(6,1);
                else
                    % Handle no joint case (world)
                    if tf_i.Parent.IsRoot
                        Xup_i = eye(6);
                        S_i = [];
                        v_i = zeros(6,1);
                        c_i = zeros(6,1);
                    else
                        % Get the joint
                        joint = tf_i.Entity.Joints;
                        [XJ,S_i,vJ] = FeatherstoneDynamics.GetJointData(joint);
                        XTree_i = FeatherstoneDynamics.FromSO3(tf_i.Local);    % Get the local transformation
                        % Local spacial joint transformation
                        Xup_i = XJ * XTree_i;
    
                        % Find parent frame properties
                        [flag,pid] = this.GetParentIndex(tf_i.Uid); 
                        if ~flag
                            error("Something went wrong, no joint parent found.");
                        end
    
                        % The parent is the world
                        if isa(joint,"FloatingJoint")
                            % Velocity is the floating joint state
                            v_0 = [body_i.AngularVelocity;body_i.Velocity];
                        else
                            % The parent any other link
                            v_0 = this.v{pid};
                        end
    
                        % Calculate the velocity terms
                        v_i = Xup_i*v_0 + vJ;
                        c_i = FeatherstoneDynamics.MotionCross(v_i) * vJ;
                    end
                end

%                 % Handle no joint case (world)
%                 if tf_i.Parent.IsRoot
%                     % Assume it is the world
%                     Xup_i = eye(6);
%                     S_i = [];
%                     v_i = zeros(6,1);
%                     c_i = zeros(6,1);
%                 else
%                     % Get the joint
%                     joint = tf_i.Entity.Joints;
%                     [XJ,S_i,vJ] = FeatherstoneDynamics.GetJointData(joint);
%                     XTree_i = FeatherstoneDynamics.FromSO3(tf_i.Local);    % Get the local transformation
%                     % Local spacial joint transformation
%                     Xup_i = XJ * XTree_i;
% 
%                     % Find parent frame properties
%                     [flag,pid] = this.GetParentIndex(tf_i.Uid); 
%                     if ~flag
%                         error("Something went wrong, no joint parent found.");
%                     end
% 
%                     % The parent is the world
%                     if isa(joint,"FloatingJoint")
%                         % Velocity is the floating joint state
%                         v_0 = [tf_i.AngularVelocity;tf_i.Velocity];
%                     else
%                         % The parent any other link
%                         v_0 = this.v{pid};
%                     end
% 
%                     % Calculate the velocity terms
%                     v_i = Xup_i*v_0 + vJ;
%                     c_i = FeatherstoneDynamics.MotionCross(v_i) * vJ;
%                 end

                % Assign to containers
                this.Xup{i} = Xup_i;
                this.S{i} = S_i;
                this.v{i} = v_i;
                this.c{i} = c_i;
            end
        end
        function [this] = ComputeForces(this,bodies)
            % This function computes the forces of all the provided
            % transforms.

            % Container setup
            n = numel(bodies);
            this.IA = cell(n,1);
            this.pA = cell(n,1);
            this.U = cell(n,1);
            this.d = cell(n,1);
            this.u = cell(n,1);

            % Calculate the inertial (force + velocity) contributions 
            for i = 1:1:n
                body_i = bodies(i);
%                 tf_i = body_i.Transform;
%                 fprintf("Calculating forces for %s.\n",transform_i.Entity.Name);
                
                v_i = this.v{i};

%                 assert(~isempty(rb_i),"Cannot handle non-rigidbody components yet.");

                % Body properties
                IA_i = FeatherstoneDynamics.Inertia(body_i.Mass,body_i.CenterOfMass,body_i.Inertia);
                % The net force and torques acting on the body
                f_i = [body_i.torqueAccumulator;body_i.forceAccumulator];                       
                % Sum the inertial-velocity and external force/torque terms
                pA_i = FeatherstoneDynamics.ForceCross(v_i) * IA_i * v_i - f_i;
                % Store properties
                this.IA{i} = IA_i;  % Always [6x6]
                this.pA{i} = pA_i;  % Always [6x1]
            end

            % Calculate the joint subspace inertial properites
            for i = n:-1:1
                body_i = bodies(i);
                tf_i = body_i.Transform;

                % Get the joint
                joint = tf_i.Entity.Joints;
                if isempty(joint)
%                     error("No joint defined for '%s'.",transform_i.Entity.Name);
                    continue;
                end

                % Transform
                Xup_i = this.Xup{i};
                c_i  = this.c{i};
                IA_i = this.IA{i};
                pA_i = this.pA{i};
                S_i  = this.S{i};
                Si_trans = transpose(S_i);

                % No joint properties to calculate
                if ~isa(joint,"MovableJoint")
                    continue;
                end

                % Get equivalent inputs
                if isa(joint,"ActuatedJoint")
                    tau_i = joint.JointInput;
                else
                    tau_i = zeros(joint.DegreesOfFreedom,1);
                end

                % Calculate inertial properties through the joint subspace
                U_i = IA_i * S_i;
                d_i = Si_trans * U_i;
                u_i = tau_i - Si_trans*pA_i;

                % Store the variables
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
        function [this] = ComputeAccelerations(this,bodies)
            % This function computes the accelerations of all the provided
            % transforms.

            n = numel(bodies);
            this.a = cell(n,1);

            for i = 1:n
                body_i = bodies(i);
                transform_i = body_i.Transform;
                
                % Handle no joint case (world)
                if transform_i.Parent.IsRoot
                    % Assume it is the world
                    a_0 = -this.g;
                else
                    % Get the joint
                    joint = transform_i.Entity.Joints;

                    % Find parent frame properties
                    [flag,pid] = this.GetParentIndex(transform_i.Uid); 
                    if ~flag
                        error("Something went wrong, no joint parent found.");
                    end

                    % The parent is the world
                    if isa(joint,"FloatingJoint")
                        % Acceleration is the base inertial acceleration 
                        a_0 = - this.IA{i} \ this.pA{i};
                    else
                        % The parent any other link
                        a_0 = this.a{pid};
                    end
                end

                % Calculate the frame acceleration
                a_i = this.Xup{i} * a_0 + this.c{i};

                joint = transform_i.Entity.Joints;
                % Add the joint acceleration 
                if ~isempty(joint) && isa(joint,"MovableJoint")
                    % Get the acceleration through the joint subspace.
                    qdd_i = (this.u{i} - this.U{i}'*a_i)/this.d{i};
                    % The resulting acceleration of the link
                    a_i = a_i + this.S{i}*qdd_i;
                    % Store the joint accelerations
                    this.qdd{i} = qdd_i;
                end

                % Record the frame acceleration 
                this.a{i} = a_i;
            end

            % Apply gravity to floaters
            for i = 1:n
                body_i = bodies(i);
                transform_i = body_i.Transform;

                if transform_i.Parent.IsRoot
                    % Skip the world
                    continue;
                end

                joint_i = transform_i.Entity.Joints;
                if isempty(joint_i)
                    continue
                end
                if joint_i.Code == JointCode.Floating
                    a_0 = - this.IA{i} \ this.pA{i};
                    this.a{i} = this.Xup{i} \ a_0 + this.g;
                end
            end
        end
        function [flag,ind,pid] = GetParentIndex(this,uid)
            % This function gets the parent properties of a given transform
            % in the flattened manner.
            % flag  - A parent was found.
            % ind   - Its index in the map
            % pid   - The parent's uid

            ind = -1;
            flag = false;

            childLogical = this.Map(:,1) == uid;

            if 0 == sum(childLogical)
                return;
            end

            pid = this.Map(childLogical,2);
            parentLogical = this.Map(:,1) == pid;

            for i = 1:numel(parentLogical)
                if parentLogical(i) == 1
                    ind = i;
                    flag = true;
                    break
                end
            end
        end
    end
    methods (Static)
        % Spacial mathmatics
        function [XJ,SJ,vJ] = GetJointData(joint)
            % This function provides a breakout from the JCalc method to
            % the joint definitions

            % Handle the null joint 
            if isempty(joint)
                XJ = eye(6);
                SJ = [];
                vJ = zeros(6,1);
                return
            end
            % Get the joint data
            Tj = joint.GetJointTransformation();
            SJ = joint.GetMotionSubspace();
            vJ = [];
            % Construct the spacial matrix
            XJ = FeatherstoneDynamics.FromTranslationRotation(Tj(1:3,4),Tj(1:3,1:3));
            % The spacial joint velocity
            if isa(joint,"MovableJoint") 
                % Sanity checking
                assert(size(SJ,1) == 6,"Motion subspace matrix rows must equate to 6.");
                assert(size(SJ,2) == joint.DegreesOfFreedom,"Motion subspace matrix columns must equate to the dof.");
                vJ = SJ*joint.JointVelocity;
            else
                % No additional joint velocity
                vJ = zeros(6,1);
            end
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
        function [M] = Inertia(m,com,I)
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
            M = [ I + m*C*C', m*C; m*C', m*eye(3) ];
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
                so3.Rotation.GetMatrix());
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