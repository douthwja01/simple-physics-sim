classdef VerletObject < CollisionObject
    %VERLETOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Mass = 1;
        Acceleration = zeros(3,1);
        Velocity = zeros(3,1);
        Position = zeros(3,1);
    end
    
    methods
        function [this] = VerletObject(varargin)
            % Verlet object constructor
            this = this@CollisionObject(varargin{:});
        end
        
        function [this] = Update(this,dt)
            % Update the physics
            this.Velocity = this.Velocity + this.Acceleration*dt;
            this.Position = this.Position + this.Velocity*dt + (1/2)*this.Acceleration*dt^2;
        end
        function [this] = PhysicsUpdate(this,dt)
            % Update the physics 
            this.Force = this.Force + this.Mass * g;
            this.Acceleration = this.Force / this.Mass;
            this.Velocity = this.Velocity + dt;
            this.Position = this.Position + this.Velocity * dt;
        end
        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.Acceleration = a;
        end

    end
end

