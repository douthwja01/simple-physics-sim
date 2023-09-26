
classdef SimObject < handle
    properties
        a = zeros(3,1);
        v = zeros(3,1);
        p = zeros(3,1);

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
            this.a = g;
        end
        function [this] = Update(this,dt)
            % Update the physics
            this.v = this.v + this.a*dt;
            this.p = this.p + this.v*dt + (1/2)*this.a*dt^2;
        end

        function [this] = Collide(this)

        end
        
        function [this] = UpdateGraphics(this,ax)
            % Draw the object in its current state
            
            if ~isa(this.Handle,"matlab.graphics.primitive.Transform")
                this.InitialiseGraphics(ax);
            end

            % Update the position
            this.Transform.position = this.p;
            
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