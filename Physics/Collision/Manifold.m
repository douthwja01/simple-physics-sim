classdef Manifold < event.EventData
    %MANIFOLD is a simple helper class the provides the data on a
    %collision between two bodies.
    
    properties
        ColliderA = Collider.empty;         % The first collisionbject
        ColliderB = Collider.empty;         % The second collision object
        Points = CollisionPoints.empty;     % The points defining the collision
    end
    
    methods
        function [this] = Manifold(a,b,points)
            % COLLISION Construct an instance of a collision.
            
            % Sanity check
            assert(isa(a,"Collider"),"Expecting a valid first collider.");
            assert(isa(b,"Collider"),"Expecting a valid second collider.");

            % Capture the properties
            this.ColliderA = a;
            this.ColliderB = b;
            % If manifold defines points
            if nargin > 2
                this.Points = points;
            end
        end
    end
end

