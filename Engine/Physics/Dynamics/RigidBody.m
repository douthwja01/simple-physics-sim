
classdef RigidBody < Element
    properties (SetAccess = private)
        Mass = 1;
        InverseMass = 1;
        Inertia = eye(3);
        InverseInertia = eye(3);
        CenterOfMass = [0.5;0;0];   % Location of the mass
        Gravity = -9.81;            % Local value of gravity (viable between instances)
        TakesGravity = true;        % Uses world gravity
        StaticFriction = 0.6;       % Static friction coefficient
        DynamicFriction = 0.8;      % Dynamic friction coefficient
        Restitution = 0.5;          % Elasticity of collisions 
% 		, AxisLock(0.f)
% 		, IsAxisLocked(0.f)
% 		SimGravity(true)
        IsDynamic = true;           % Has dynamic reactions
		IsSimulated = true;         % Is capable of movement
    end

    properties (SetAccess = private)
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
    end

    methods
        function [this] = ApplyForce(this,f,p)
            % Sanity check one
            assert(IsColumn(f,3),"Expecting a valid 3D force vector.");
            % Update the orce accumulator
            this.NetForce = this.NetForce + f;
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
            this.NetTorque = this.NetTorque + tau;
        end
        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.Transform.Acceleration = a;
        end
    end
end