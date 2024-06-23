
classdef RigidBody < Element


    properties %(SetAccess = private)
        Mass = 1;
        InverseMass = 1;
        Inertia = eye(3);
        InverseInertia = eye(3);
        CenterOfMass = [0.5;0;0];   % Location of the mass
        StaticFriction = 0.6;       % Static friction coefficient
        DynamicFriction = 0.8;      % Dynamic friction coefficient
        Restitution = 0.5;          % Elasticity of collisions 
        IsDynamic = true;           % Has dynamic reactions
		IsSimulated = true;         % Is capable of movement
        % Legacy?
        Gravity = -9.81;            % Local value of gravity (viable between instances)
        TakesGravity = true;        % Uses world gravity
% 		, AxisLock(0.f)
% 		, IsAxisLocked(0.f)
% 		SimGravity(true)
    end

    properties (SetAccess = private)
        forceAccumulator = zeros(3,1);
        torqueAccumulator = zeros(3,1);
        IsSimulated = true;         % Is capable of movement
        IsDynamic = true;           % Has dynamic reactions
        IsStatic = false;
        % Dynamic properties
        LinearVelocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
        LinearAcceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
        LinearMomentum = zeros(3,1);
        AngularMomentum = zeros(3,1);
    end

    properties (Access = private)
        NetForce = zeros(3,1);
        NetTorque = zeros(3,1);
    end

    methods
        function [this] = RigidBody(entity)
            % CONSTRUCTOR - Construct a copy of the rigidbody class.

            % Input check
            if nargin < 1
                entity = Entity.empty;
            end

            % Rigidbody object constructor
            [this] = this@Element(entity);
        end
        % Get/sets
        function set.IsDynamic(this,isDynamic)
            assert(islogical(isDynamic),"Expecting a boolean is dynamic.");
            this.IsDynamic = isDynamic;
        end
        function set.IsSimulated(this,isSimulated)
            assert(islogical(isSimulated),"Expecting a boolean is simulated.");
            this.IsSimulated = isSimulated;
        end
        function set.StaticFriction(this,Csf)
            assert(isscalar(Csf),"Expecting a scalar static friction coefficient.");
            this.StaticFriction = Csf;
        end
        function set.DynamicFriction(this,Cdf)
            assert(isscalar(Cdf),"Expecting a scalar dynamic friction coefficient.");
            this.DynamicFriction = Cdf;
        end
        function set.Restitution(this,Cr)
            assert(isscalar(Cr),"Expecting a scalar restitution coefficient.");
            this.Restitution = Cr;
        function set.IsStatic(this,s)
            assert(islogical(s),"Expecting a valid logical IsStatic flag.");
            this.IsStatic = s;
        end        
        function set.LinearVelocity(this,v)
            assert(IsColumn(v,3),"Expecting a valid Cartesian linear velocity [3x1].");
            this.LinearVelocity = v;
        end
        function set.AngularVelocity(this,w)
            assert(IsColumn(w,3),"Expecting a valid Cartesian angular velocity [3x1].");
            this.AngularVelocity = w;
        end 
        function set.LinearAcceleration(this,dv)
            assert(IsColumn(dv,3),"Expecting a valid Cartesian linear acceleration [3x1].");
            this.LinearAcceleration = dv;
        end  
        function set.AngularAcceleration(this,dw)
            assert(IsColumn(dw,3),"Expecting a valid Cartesian angular acceleration [3x1].");
            this.AngularAcceleration = dw;
        end 
        function set.LinearMomentum(this,m)
            assert(IsColumn(m,3),"Expecting a valid Cartesian linear momentum [3x1].");
            this.LinearMomentum = m;
        end
        function set.AngularMomentum(this,m)
            assert(IsColumn(m,3),"Expecting a valid Cartesian angular momentum [3x1].");
            this.AngularMomentum = m;
        end
    end
    % Rigidbody interfaces
    methods
        function [this] = ApplyForce(this,f,p)
            % Sanity check one
            assert(IsColumn(f,3),"Expecting a valid 3D force vector.");
            % Update the orce accumulator
            this.forceAccumulator = this.forceAccumulator + f;
            if nargin < 3
                return;
            end
            % Apply a force 'f' at position 'p' on the body.
            assert(IsColumn(p,3),"Expecting a valid 3D position vector.");
            % Create a torque
            this.ApplyTorque(cross(p,f));
        end
        function [this] = ApplyTorque(this,tau)
            assert(IsColumn(tau,3),"Expecting a valid 3D torque vector.");
            this.torqueAccumulator = this.torqueAccumulator + tau;
        end
        function [this] = ClearAccumulators(this)
            % This function clears the accumulators defining the forces and
            % torques acting on the body are defined.

            this.forceAccumulator = zeros(3,1);
            this.torqueAccumulator = zeros(3,1);
        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.LinearAcceleration = a;
        end
    end
end