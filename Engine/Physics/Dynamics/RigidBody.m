% This c
classdef RigidBody < Particle


    properties %(SetAccess = private)
        Mass = 1;
        InverseMass = 1;
        Inertia = eye(3);
        InverseInertia = eye(3);
        Gravity = -9.81;            % Local value of gravity (viable between instances)
        TakesGravity = true;        % Uses world gravity
%         StaticFriction = 0.6;       % Static friction coefficient
%         DynamicFriction = 0.8;      % Dynamic friction coefficient
%         Restitution = 0.5;          % Elasticity of collisions 
% 		, AxisLock(0.f)
% 		, IsAxisLocked(0.f)
% 		SimGravity(true)
        IsDynamic = true;           % Has dynamic reactions
%         IsStatic = false;
        % Dynamic properties
        LinearAcceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
        LinearMomentum = zeros(3,1);
        AngularMomentum = zeros(3,1);
    end

    properties (Access = private)
        NetForce = zeros(3,1);
        NetTorque = zeros(3,1);
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

    methods
        function [this] = ApplyForce(this,f,p)
            % Sanity check one
            assert(IsColumn(f,3),"Expecting a valid 3D force vector.");
            % Update the orce accumulator
            this.NetForce = this.NetForce + f;
            if nargin < 3
                return;
            end
            % Apply a force 'f' at position 'p' on the body.
            assert(IsColumn(p,3),"Expecting a valid 3D position vector.");
            % Create a torque
            this.ApplyTorque(cross(p,f));
        end
        function [this] = ApplyTorque(this,tau)
            assert(IsColumn(tau,3),"Expecting a valid 3D torque vector.");
            this.NetTorque = this.NetTorque + tau;
        end
        function [this] = Accelerate(this,a)
            % Calcuate the acceleration
            this.LinearAcceleration = a;
        end
    end
end