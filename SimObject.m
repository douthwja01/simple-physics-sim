
classdef SimObject < handle
    properties
        Radius = 0.5;

        acceleration = zeros(3,1);
        velocity = zeros(3,1);
        position = zeros(3,1);
        Handle = gobjects(1);
    end
    properties (Access = private)
        Transform;
    end

    methods
        function [this] = SimObject()
            % Object constructor

            % Populate the transform
            this.Transform = Transform();
        end

        function [this] = ApplyGravity(this,g)
            % Calcuate the acceleration
            this.acceleration = g;
        end
        function [this] = Update(this,dt)
            % Update the physics
            this.velocity = this.velocity + this.acceleration*dt;
            this.position = this.position + this.velocity*dt + (1/2)*this.acceleration*dt^2;
        end

        function [this] = Collide(this)

        end
        
        function [this] = UpdateGraphics(this,ax)
            % Draw the object in its current state
            
            if ~isa(this.Handle,"matlab.graphics.primitive.Transform")
                this.InitialiseGraphics(ax);
            end

            % Update the position
            this.Transform.position = this.position;
            
            % Transform plotf
            set(this.Handle,"Matrix",this.Transform.transform);
        end
    end
    methods (Access = private)
        function [this] = InitialiseGraphics(this,ax)
            % Plot the handle
            h_data = this.Transform.Plot(ax);

            h_sph = VisualUtilities.DrawSphere(1,h_data);
            set(h_sph,"FaceColor","r");
            this.Handle = h_data;
        end
    end
end