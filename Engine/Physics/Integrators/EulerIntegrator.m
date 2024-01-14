classdef EulerIntegrator < Integrator
    %EULERINTEGRATOR - An integrator element based on the Euler method.

    properties (Constant)
        Name = "Forward Euler-method";
    end
    methods (Access = protected)
        function [this] = IntegrateTransform(this,pose,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            p0 = pose.GetWorldPosition();
            q0 = pose.GetWorldRotation();
            v0 = pose.Velocity;
            w0 = pose.AngularVelocity;

            % Compute the simple Forward-Euler step
            v = v0 + pose.Acceleration*dt;
            w = w0 + pose.AngularAcceleration*dt;
            
            % Computer 
            dq = PhysicsExtensions.qDifferential(q0,w);

            % Euler step
            rotation = q0 + dq*dt;
            position = p0 + v*dt;

            pose.Velocity = v;
            pose.AngularVelocity = w;
            % Set the world properties
            pose.SetWorldPosition(position);
            pose.SetWorldRotation(rotation);
        end
    end
end