classdef Particle < Element
    %PARTICLE is the basic kinematic model for objects with motion
    %capabilities.

    properties
        % Friction
        StaticFriction = 0.6;       % Static friction coefficient
        DynamicFriction = 0.8;      % Dynamic friction coefficient
        Restitution = 0.5;          % Elasticity of collisions
        % Kinematic properties
        IsStatic = false;           % Is capable of movement
        LinearVelocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
    end
    properties (SetAccess = protected,Hidden)
        linearDamping = 0.99;
        angularDamping = 0.5;
        forceAccumulator = zeros(3,1);
        torqueAccumulator = zeros(3,1);
        linearImpulseAccumulator = zeros(3,1);
        angularImpulseAccumulator = zeros(3,1);
    end
    methods
        function [this] = Particle(entity)
            %PARTICLE - Construct an instance of the particle class

            % Input check
            if nargin < 1
                entity = Entity.empty;
            end

            % Rigidbody object constructor
            [this] = this@Element(entity);
        end
        % Get/Sets
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
    end
    %% Dynamics
    methods
        function [this] = ApplyForce(this,f,p)
            % Sanity check one
            assert(IsColumn(f,3),"Expecting a valid 3D force vector.");
            % Update the orce accumulator
            this.forceAccumulators = this.forceAccumulators + f;
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
            this.torqueAccumulators = this.torqueAccumulators + tau;
        end
    end

    %% Utilities
    methods
        function [this] = CheckAwake(this)
            % This function checks if the body needs to be awake via a
            % minimum frame movement.

            % Determine if rigid body is awake
            d = this.lastWorldPosition - Transform.GetWorldPosition();

            d2 = d*d; %Mask(Mul(d, d), Constants::MaskOffW);

            maxMovement = max(d2);

            if maxMovement < 1E-4 %&& ~this.IsAlwaysAwake()
                this.SetAwake(false);
            else
                this.SetAwake(true);
            end
        end
        function [this] = ClearAccumulators(this)
            % This function clears all the dynamic properties of the
            % particle for the current frame to allow them to by
            % dynamically rederrived in the next frame.

            % Clear the dynamic containers
            this.forceAccumulator = zeros(3,1);
            this.torqueAccumulator = zeros(3,1);
        end
    end
end

