
classdef MeshRenderer < Renderer
    % This class is a an element responsible for the appearance of the
    % visual elements of an Entity.

    properties
        Base = Mesh.empty;
        Colour = 'r';
        Alpha = 0.5;
    end
    properties (Access = private)
        Handle = gobjects(1);
        MeshHandle = gobjects(1);
    end
    methods
        function [this] = MeshRenderer()
            % Constructor for a visual elements

            % Assign the entity
            [this] = this@Renderer();
        end
        % Get/sets
        function set.Base(this,m)
            assert(isa(m,"Mesh"),"Expecting a valid base-mesh object.");
            this.Base = m;
        end
        function set.Colour(this,colorString)
            assert(isstring(colorString),"Expecting a valid color code string (i.e 'r').");
            this.Colour = colorString;
            this.RenderPropertiesUpdated();
        end
        function set.Alpha(this,alpha)
            assert(isscalar(alpha),"Expecting a valid alpha value.");
            this.Alpha = alpha;
            this.RenderPropertiesUpdated();
        end
    end

    methods
        function [this] = Update(this,ax)
            % Draw the object in its current state
            
            if ~isa(this.Handle,"matlab.graphics.primitive.Transform")
                this.Initialise(ax);
            end

            % Transform plot
            %m = this.Transform.GetWorldMatrix();
%             set(this.Handle,"Matrix",m);
            set(this.Handle,"Matrix",this.Transform.Inertial.GetMatrix());
        end
    end
    % Utilities
    methods (Access = protected)
        function [this] = Initialise(this,ax)
            % Initialise the renderer.

            % Plot the handle
            this.Handle = this.Entity.Transform.Plot(ax);

            % Sanity check
            if isempty(this.Base)
                wwarning("No mesh assigned to renderer.");
                return;
            end

            % Get the transformed mesh
            mesh = this.GetTransformedMesh();
            % Plot the mesh to the handle
            this.MeshHandle = mesh.Draw(this.Handle);
            % Render property assignment
            this.RenderPropertiesUpdated();
        end
        function [mesh] = GetTransformedMesh(this)
            % Convert the default mesh to the rendered mesh.
            % (Overridden in parent classes).

            mesh = this.Base;           
        end
        function [this] = RenderPropertiesUpdated(this)
            % Update any 'one-time' mesh properties against the active mesh
            % handle if there is one.
            
            % If mesh-handle present
            if isa(this.Handle,"matlab.graphics.GraphicsPlaceholder")
                return;
            end
            % Update any 'one-time' properties associated with the mesh
            set(this.MeshHandle,"FaceAlpha",this.Alpha);
            set(this.MeshHandle,"FaceColor",this.Colour);
        end
    end
end