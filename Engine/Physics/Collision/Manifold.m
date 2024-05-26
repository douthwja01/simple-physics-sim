classdef Manifold < event.EventData
    %MANIFOLD is a simple helper class the provides the data on a
    %collision between two bodies.
    
    properties
        ColliderA = Collider.empty;         % The first collisionbject
        ColliderB = Collider.empty;         % The second collision object
        BodyA = RigidBody.empty;
        BodyB = RigidBody.empty;
        Points = ContactPoints.empty;       % The points defining the collision
    end
    
    methods
        function [this] = Manifold(colliderA,colliderB,points)
            % COLLISION Construct an instance of a collision.
            
            % Sanity check
            assert(isa(colliderA,"Collider"),"Expecting a valid first collider.");
            assert(isa(colliderB,"Collider"),"Expecting a valid second collider.");

            % If manifold defines points
            if nargin > 3
                this.Points = points;
            end

            % Capture the properties
            this.ColliderA = colliderA;
            this.ColliderB = colliderB;
            % Get the rigidbody references
            this.BodyA = this.ColliderA.Entity.RigidBody;
            this.BodyB = this.ColliderB.Entity.RigidBody;
        end
    end
end

