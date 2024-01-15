classdef EntityCreator
    %ENTITYCREATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods (Static)
        function [plane] = Plane(position,rotation)
            % This function creats a plane object.

            plane = Empty("Plane",position,rotation);
            plane.Pose.IsStatic = true;
            plane.Pose.SetWorldScale([10;10;0.1]);
            % Add plane elements
            plane.Collider = PlaneCollider();
            plane.Renderer = PlaneRenderer();
        end
        function [cube] = Box(position,rotation)
            % This function creates a box object.

            % Entity box
            cube = Empty("Box",position,rotation);
            % Add box elements
            cube.Renderer = BoxRenderer();
            cube.Collider = BoxCollider();
        end
        function [sphere] = Sphere(position,rotation)
            % This function creates a sphere entity.

            % Template
            sphere = Empty("Sphere",position,rotation);
            % Add box elements
            sphere.Renderer = SphereRenderer();
            sphere.Collider = SphereCollider();
        end
        function [obj] = Empty(name,position,rotation)
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
            obj = Entity(name);
            % Set the entity location
            obj.Pose.SetWorldPosition(position);
            obj.Pose.SetWorldRotation(rotation);
        end
    end
end

