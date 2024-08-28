classdef Particle < Element
    %PARTICLE is the basic kinematic model for objects with motion
    %capabilities.
    
    properties        
        % Friction
        StaticFriction = 0.6;       % Static friction coefficient
        DynamicFriction = 0.8;      % Dynamic friction coefficient
        Restitution = 0.5;          % Elasticity of collisions 
        % Kinematic properties
        IsStatic = false;           % Is capable of movement
        LinearVelocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
    end
    
    methods
        function [this] = Particle(entity)
            %PARTICLE - Construct an instance of the particle class

            % Input check
            if nargin < 1
                entity = Entity.empty;
            end

            % Rigidbody object constructor
            [this] = this@Element(entity);
        end
        % Get/Sets        
        function set.IsStatic(this,s)
            assert(islogical(s),"Expecting a valid logical IsStatic flag.");
            this.IsStatic = s;
        end   
        function set.LinearVelocity(this,v)
            assert(IsColumn(v,3),"Expecting a valid Cartesian linear velocity [3x1].");
            this.LinearVelocity = v;
        end
        function set.AngularVelocity(this,w)
            assert(IsColumn(w,3),"Expecting a valid Cartesian angular velocity [3x1].");
            this.AngularVelocity = w;
        end 
    end
end

