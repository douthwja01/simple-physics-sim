classdef PlaneRenderer < MeshRenderer
    % This class is an element responsible for the rendering of planes
    % for simple visualisation of an Entity.
    
    properties 
        Normal = [1;0;0];
        Width = 1;
        Depth = 1;
    end
    methods
        function [this] = PlaneRenderer()
            % CONSTRUCTOR - Construct an instance of the plane renderer.

            % Assign the entity
            [this] = this@MeshRenderer();
            % The unit presentation
            this.Base = MeshExtensions.UnitPlane(); 
        end
        % Get/sets
        function set.Normal(this,n)
            assert(IsColumn(n,3),"Expecting a valid normal vector [3x1].");
            this.Normal = n;
        end
    end
    methods (Access = protected)
        function [mesh] = GetTransformedMesh(this)
            % Get the mesh transformed by the parameters of the plane
            % renderer.

            % Get the base mesh
            mesh = GetTransformedMesh@MeshRenderer(this);
            % Get the transformed mesh
            mesh = mesh.ScaleBy(this.Width,this.Depth,1);
        end
    end
end