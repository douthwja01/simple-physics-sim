classdef CollisionObject < Element
    %COLLISIONOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        Collider;
        IsDynamics = true;
        IsTrigger = false;
    end
    methods
        function [this] = CollisionObject(varargin)
            % Collision object constructor
            this = this@Element(varargin{:});
            % Collider
            this.Collider = SphereCollider();
        end
        % Get/sets
        function [this] = SetDynamic(this,isDynamics)
            assert(islogical(isDynamics),"Expecting a boolean is dynamic.");
            this.IsDynamics = isDynamics;
        end
        function [this] = SetTrigger(this,isTrigger)
            assert(islogical(isTrigger),"Expecting a boolean trigger.");
            this.IsTrigger = isTrigger;
        end
    end
    events
        % We want to register a callback when a collision occurs.
        OnCollision;
    end
end

