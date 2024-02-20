classdef BoxRenderer < MeshRenderer
    %BOXRENDERER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Height = 1;
        Width = 1;
        Depth = 1;
    end
    methods
        function [this] = BoxRenderer()
            % CONSTRUCTOR - Construct an instance of the box renderer.
            
            % Assign the entity
            [this] = this@MeshRenderer();
            % Default to unit cuboid
            this.Base = MeshExtensions.UnitCube(); 
        end
    end
    methods (Access = protected)
        function [mesh] = GetTransformedMesh(this)
            % Convert the default mesh to the rendered mesh.
            % (Overridden in parent classes).

            % Get the base mesh
            mesh = GetTransformedMesh@MeshRenderer(this); 
            % Get the transformed mesh
            mesh = mesh.ScaleBy(this.Width,this.Height,this.Depth);
        end
    end
end

