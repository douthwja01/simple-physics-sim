classdef BoxRenderer < MeshRenderer
    %BOXRENDERER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function [this] = BoxRenderer(entity)
            % Constructor for a visual elements

            % Assign the entity
            [this] = this@MeshRenderer(entity);

            % Default to unit cuboid
            this.Mesh = MeshExtensions.UnitCube(); 
        end
    end
end

