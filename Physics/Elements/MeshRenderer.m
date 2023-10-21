
classdef MeshRenderer < Element
    % This class is a an element responsible for the appearance of the
    % visual elements of an Entity.

    properties %(SetAccess = protected)
        Mesh = Mesh.empty;
        Colour = 'r';
        Alpha = 0.5;
    end
    properties (Access = private)
        Transform;
        Handle = gobjects(1);
        MeshHandle = gobjects(1);
    end
    methods
        function [this] = MeshRenderer()
            % Constructor for a visual elements
        end
        % Get/sets
        function set.Mesh(this,m)
            assert(isa(m,"Mesh"),"Expecting a valid mesh object.");
            this.Mesh = m;
        end
        function set.Colour(this,colorString)
            assert(isstring(colorString),"Expecting a valid color code string (i.e 'r').");
            this.Colour = colorString;
            if ~isa(this.Handle,"matlab.graphics.GraphicsPlaceholder")
                set(this.MeshHandle,"FaceColor",this.Colour);
            end
        end
        function set.Alpha(this,alpha)
            assert(isscalar(alpha),"Expecting a valid alpha value.");
            this.Alpha = alpha;
            if ~isa(this.Handle,"matlab.graphics.GraphicsPlaceholder")
                set(this.MeshHandle,"FaceAlpha",this.Alpha);
            end
        end
    end

    methods
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

            % Sanity check
            if isempty(this.Mesh)
                warning("No mesh assigned to renderer.");
                return;
            end

            % Plot the mesh to the handle
            this.MeshHandle = this.Mesh.Draw(h_data);
            % Set the colour
            set(this.MeshHandle,"FaceColor",this.Colour);
            set(this.MeshHandle,"FaceAlpha",this.Alpha);
            this.Handle = h_data;
        end
    end
end