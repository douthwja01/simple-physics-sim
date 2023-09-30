classdef CollisionObject < SimObject
    %COLLISIONOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Radius = 0.5;
        Collider;
    end
    
    methods
        function [this] = CollisionObject(varargin)
            % Collision object constructor
            this = this@SimObject(varargin{:});
        end
    end
end

