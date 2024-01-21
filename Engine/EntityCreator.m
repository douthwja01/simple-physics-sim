classdef EntityCreator
    %ENTITYCREATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods (Static)
        function [plane] = Plane(name,position,rotation)
            % This function creats a plane object.

            % Input parsing
            if nargin < 3
                rotation = [1;0;0;0];
            end
            if nargin < 2
                position = zeros(3,1);
            end
            if nargin < 1
                name = "Plane";
            end

            % Create the empty
            plane = EntityCreator.Empty(name,position,rotation);
            % Add plane elements
            plane.Collider = PlaneCollider();
            plane.Renderer = PlaneRenderer();
        end
        function [cube] = Box(name,position,rotation)
            % This function creates a box object.

            % Input parsing
            if nargin < 3
                rotation = [1;0;0;0];
            end
            if nargin < 2
                position = zeros(3,1);
            end
            if nargin < 1
                name = "Box";
            end
            
            % Entity box
            cube = EntityCreator.Empty(name,position,rotation);
            % Add box elements
            cube.Renderer = BoxRenderer();
            cube.Collider = BoxCollider();
        end
        function [sphere] = Sphere(name,position,rotation)
            % This function creates a sphere entity.

            % Input parsing
            if nargin < 3
                rotation = [1;0;0;0];
            end
            if nargin < 2
                position = zeros(3,1);
            end
            if nargin < 1
                name = "Sphere";
            end

            % Template
            sphere = EntityCreator.Empty(name,position,rotation);
            % Add box elements
            sphere.Renderer = SphereRenderer();
            sphere.Collider = SphereCollider();
        end
        function [this] = Empty(name,position,rotation)
            % Create a simple entity with a given name, position and
            % rotation.

            if nargin < 3
                rotation = [1;0;0;0];
            end
            if nargin < 2
                position = zeros(3,1);
            end
            if nargin < 1
                name = "Empty";
            end
            
            % Entity box
            this = Entity(name);
            % Set the entity location
            this.Transformation.SetWorldPosition(position);
            this.Transformation.SetWorldRotation(rotation);
        end
    end
end

