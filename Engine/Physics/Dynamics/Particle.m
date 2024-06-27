classdef Particle < Element
    %PARTICLE is the basic kinematic model for objects with motion
    %capabilities.Particles are modelled as point masses with limited
    %dynamic interaction.

    properties
        % Mass
        Mass = 1;
        InverseMass = 1;
        
        % Friction
        StaticFriction = 0.6;       % Static friction coefficient
        DynamicFriction = 0.8;      % Dynamic friction coefficient
        Restitution = 0.5;          % Elasticity of collisions
        % Kinematic properties
        IsSimulated = true;         % Is the result of simulated motion
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
        function set.Mass(this,m)
            assert(isscalar(m),"Expecting a valid scalar mass (kg).");
            this.Mass = m;
        end
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
    methods
        function [this] = Integrate(this,dt)

            if this.IsStatic
                return;
            else
                %m_derivedValid = false;
            end

            orientation = Transform.GetOrientationParentSpace();
            position    = Transform.GetPositionParentSpace();

            %orientation = QuatAddScaled(orientation, m_angularVelocity, timeStep);
            dQ = Quaternion.Rate(orientation, this.AngularVelocity);
            orientation = orientation + dQ*dt;

            position = position + this.LinearVelocity*dt;

            this.Transform.SetOrientation(orientation); % Set orientation in parent space
            this.Transform.SetPosition(position);       % Set position in parent space

            acceleration = this.LinearAcceleration;
            acceleration = acceleration + this.forceAccumulator*this.inverseMass;
            angularAcceleration = this.inverseInertiaTensor*this.torqueAccumulator;

            this.LinearVelocity  = this.LinearVelocity  + this.linearImpulseAccumulator*this.inverseMass;
            this.AngularVelocity = this.AngularVelocity + this.angularImpulseAccumulator*this.inverseMass;

            this.LinearVelocity  = this.LinearVelocity  + acceleration*dt;
            this.AngularVelocity = this.AngularVelocity + angularAcceleration*dt;

            this.AngularVelocity = this.AngularVelocity*pow(this.angularDamping, dt);
            this.LinearVelocity  = this.LinearVelocity*pow(this.linearDamping, dt);

            for i = 1:this.Transform.NumberOfChildren
                m_children(i).Integrate(timeStep);
            end

            this.LinearImpulseAccumulator = zeros(3,1);
            this.AngularImpulseAccumulator = zeros(3,1);
        end
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

