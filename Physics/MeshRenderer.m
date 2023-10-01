
classdef MeshRenderer < Element
    % This class is a an element responsible for the appearance of the
    % visual elements of an Entity.

    properties (SetAccess = protected)
        Mesh;
        Handle = gobjects(1);
    end
    properties (Access = private)
        Transform;
    end
    methods
        function [this] = MeshRenderer()
            % Constructor for a visual elements
        end
        function [this] = Update(this,ax)
            % Draw the object in its current state
            
            if ~isa(this.Handle,"matlab.graphics.primitive.Transform")
                this.Initialise(ax);
            end

            % Transform plot
            set(this.Handle,"Matrix",this.Transform.transform);
        end
    end
    % Utilities
    methods (Access = private)
        function [this] = Initialise(this,ax)
            % Initialise the renderer.

            this.Transform = this.Entity.GetElement("Transform");

            % Plot the handle
            h_data = this.Transform.Plot(ax);

            radius = 1; %this.Collider.Radius;

            h_sph = Graphics.DrawSphere(radius,h_data);
            set(h_sph,"FaceColor","r");
            this.Handle = h_data;
        end
    end
end