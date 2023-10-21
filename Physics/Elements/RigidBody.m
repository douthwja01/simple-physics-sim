
classdef RigidBody < Element
    properties (SetAccess = private)
        Force = zeros(3,1);
        
        Mass = 1;
        IsDynamic = true;
        % Gravity
        gravity;                % Local value of gravity (viable between instances)
        takesGravity = true;    % Uses world gravity
        staticFriction = 0;     % Static friction coefficient
        dynamicFriction = 0;    % Dynamic friction coefficient
        restitution = 0.5;      % Elasticity of collisions 
    end

    properties (Access = private)
        transform;
        acceleration = zeros(3,1);
    end

    methods
        function [this] = RigidBody(varargin)
            % Rigidbody object constructor
            this = this@Element(varargin{:});
        end
        
        function [this] = Update(this)
            % Update the object using the verlet method.

            % Get the cache
            if isempty(this.transform)
               this.transform = this.Entity.GetElement("Transform"); 
            end
            
            % Update the physics
            tf = this.transform;
            tf.Acceleration = this.acceleration;
        end

        % Get/sets
        function [this] = SetDynamic(this,isDynamic)
            assert(islogical(isDynamic),"Expecting a boolean is dynamic.");
            this.IsDynamic = isDynamic;
        end
        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.acceleration = a;
        end
    end
end

%         function [this] = PhysicsUpdate(this,dt)
%             % Update the physics 
%             this.Force = this.Force + this.Mass * g;
% 
%             % Get the cache
%             if isempty(this.transform)
%                this.transform = this.Entity.GetElement("Transform"); 
%             end
%             % Update the physics
%             tf = this.transform;
%             tf.Acceleration = this.Force / this.Mass;
%             tf.Velocity = tf.Velocity + dt;
%             tf.Position = tf.Position + tf.Velocity * dt;
%         end