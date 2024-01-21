classdef SphereRenderer < MeshRenderer
    %SPHERERENDERER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Radius = 1;
    end
    methods
        function [this] = SphereRenderer()
            % CONSTRUCTOR - Construct an instance of the sphere renderer.
            
            % Assign the entity
            [this] = this@MeshRenderer();

            % Default to unit cuboid
            this.Base = MeshExtensions.UnitSphere(); 
        end
    end
    methods (Access = protected)
        function [mesh] = GetTransformedMesh(this)
            % Convert the default mesh to the rendered mesh.
            % (Overridden in parent classes).
            
            % Get the base mesh
            mesh = GetTransformedMesh@MeshRenderer(this);
            % Scale the mesh by the radius
            r = this.Radius;
            mesh = mesh.ScaleBy(r,r,r);
        end
    end
end

