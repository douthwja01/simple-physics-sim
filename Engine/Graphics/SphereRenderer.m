classdef SphereRenderer < MeshRenderer
    %SPHERERENDERER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function [this] = SphereRenderer(entity)
            % Constructor for a visual elements

            % Assign the entity
            [this] = this@MeshRenderer(entity);

            % Default to unit cuboid
            this.Mesh = MeshExtensions.UnitSphere(); 
        end
    end
end

