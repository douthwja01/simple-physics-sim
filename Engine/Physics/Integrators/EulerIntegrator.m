classdef EulerIntegrator < Integrator
    %EULERINTEGRATOR - An integrator element based on the Euler method.

    properties (Constant)
        Name = "Forward Euler-method";
    end
    methods (Access = protected)
        function [this] = IntegrateTransform(this,transform,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            so3Inertial = transform.Inertial;

            p0 = so3Inertial.Position;
            q0 = so3Inertial.Rotation;

            v0 = transform.Velocity;
            w0 = transform.AngularVelocity;

            % Compute the simple Forward-Euler step
            v = v0 + transform.Acceleration*dt;
            w = w0 + transform.AngularAcceleration*dt;
            
            % Computer 
            dq = Quaternion.Rate(q0,w);

            % Euler step
            rotation = q0 + dq*dt;
            position = p0 + v*dt;

            % Set the new velocities
            transform.Velocity = v;
            transform.AngularVelocity = w;
            % Set the new pose
            so3Inertial.Position = position;
            so3Inertial.Rotation = rotation;
        end
    end
end