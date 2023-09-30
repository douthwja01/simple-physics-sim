classdef VerletObject < CollisionObject
    %VERLETOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
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

        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.Acceleration = a;
        end

        function [this] = UpdateGraphics(this,ax)
            % Draw the object in its current state
            
            if ~isa(this.Handle,"matlab.graphics.primitive.Transform")
                this.InitialiseGraphics(ax);
            end

            % Update the position
            this.Transform.position = this.Position;
            
            % Transform plotf
            set(this.Handle,"Matrix",this.Transform.transform);
        end
    end
    methods (Access = private)
        function [this] = InitialiseGraphics(this,ax)
            % Plot the handle
            h_data = this.Transform.Plot(ax);

            h_sph = Graphics.DrawSphere(this.Radius,h_data);
            set(h_sph,"FaceColor","r");
            this.Handle = h_data;
        end
    end
end

