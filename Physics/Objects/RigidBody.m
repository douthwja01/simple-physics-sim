
classdef RigidBody < Element
    properties (SetAccess = private)
        Force = zeros(3,1);
        Acceleration = zeros(3,1);
        Velocity = zeros(3,1);
        Mass= 1;
        IsDynamic = true;
        % Gravity
        gravity;                % Local value of gravity (viable between instances)
        takesGravity = true;    % Uses world gravity
        staticFriction = 0;     % Static friction coefficient
        dynamicFriction = 0;    % Dynamic friction coefficient
        restitution = 0.5;      % Elasticity of collisions 
    end
    methods
        function [this] = RigidBody(varargin)
            % Rigidbody object constructor
            this = this@Element(varargin{:});
        end
        % Get/sets
        function [this] = SetDynamic(this,isDynamic)
            assert(islogical(isDynamic),"Expecting a boolean is dynamic.");
            this.IsDynamic = isDynamic;
        end
        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.Acceleration = a;
        end
    end

    methods %(Abstract)
        function [this] = Update(this,dt)
            % Update the physics

            transform_i = this.Entity.GetElement("Transform");

            this.Velocity = this.Velocity + this.Acceleration*dt;
            % Update the transform position
            transform_i.position = transform_i.position + this.Velocity*dt + (1/2)*this.Acceleration*dt^2;
        end
        function [this] = PhysicsUpdate(this,dt)
            % Update the physics 
            this.Force = this.Force + this.Mass * g;
            this.Acceleration = this.Force / this.Mass;
            this.Velocity = this.Velocity + dt;
            this.Position = this.Position + this.Velocity * dt;
        end
    end
end