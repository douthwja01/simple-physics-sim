% This class represents the rigidbody model for entities within the
% simulation framework.

classdef RigidBody < Particle
    properties %(SetAccess = private)
        CenterOfMass = [1;0;0];
        Inertia = eye(3);
        InverseInertia = eye(3);
        Gravity = -9.81;            % Local value of gravity (viable between instances)
        TakesGravity = true;        % Uses world gravity
% 		, AxisLock(0.f)
% 		, IsAxisLocked(0.f)
% 		SimGravity(true)
        IsDynamic = true;           % Has dynamic reactions
        % Dynamic properties
        LinearAcceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
        LinearMomentum = zeros(3,1);
        AngularMomentum = zeros(3,1);
    end

    methods
        function [this] = RigidBody(entity)
            % CONSTRUCTOR - Construct a copy of the rigidbody class.

            % Input check
            if nargin < 1
                entity = Entity.empty;
            end

            % Rigidbody object constructor
            [this] = this@Particle(entity);
        end
        % Get/sets
        function set.IsDynamic(this,isDynamic)
            assert(islogical(isDynamic),"Expecting a boolean is dynamic.");
            this.IsDynamic = isDynamic;
        end
        function set.LinearAcceleration(this,dv)
            assert(IsColumn(dv,3),"Expecting a valid Cartesian linear acceleration [3x1].");
            this.LinearAcceleration = dv;
        end  
        function set.AngularAcceleration(this,dw)
            assert(IsColumn(dw,3),"Expecting a valid Cartesian angular acceleration [3x1].");
            this.AngularAcceleration = dw;
        end 
        function set.LinearMomentum(this,m)
            assert(IsColumn(m,3),"Expecting a valid Cartesian linear momentum [3x1].");
            this.LinearMomentum = m;
        end
        function set.AngularMomentum(this,m)
            assert(IsColumn(m,3),"Expecting a valid Cartesian angular momentum [3x1].");
            this.AngularMomentum = m;
        end
    end
end