classdef Collision
    %COLLISION is a simple helper class the provides the data on a
    %collision between two bodies.
    
    properties
        ColliderA = Collider.empty;         % The first collisionbject
        ColliderB = Collider.empty;         % The second collision object
        Points = CollisionPoints.empty;    % The points defining the collision
    end
    
    methods
        function [this] = Collision(a,b,points)
            % COLLISION Construct an instance of a collision.
            
            % Capture the properties
            this.ColliderA = a;
            this.ColliderB = b;
            this.Points = points;
        end
    end
end

