
classdef MeshRenderer < Element
    % This class is a an element responsible for the appearance of the
    % visual elements of an Entity.

    properties (SetAccess = protected)
        Mesh;
        Colour = 'r';
        Handle = gobjects(1);
    end
    properties (Access = private)
        Transform;
        MeshHandle = gobjects(1);
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
        function [this] = SetColour(this,colorString)
            assert(isstring(colorString),"Expecting a valid color code string (i.e 'r').");
            this.Colour = colorString;
            if isa(this.Handle,"GraphicsPlaceholder")
                set(this.MeshHandle,"FaceColor",this.Colour);
            end
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

            this.MeshHandle = Graphics.DrawSphere(radius,h_data);
            set(this.MeshHandle,"FaceColor",this.Colour);
            this.Handle = h_data;
        end
    end
end