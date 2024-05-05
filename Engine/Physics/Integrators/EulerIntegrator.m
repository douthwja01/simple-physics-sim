classdef EulerIntegrator < Integrator
    %EULERINTEGRATOR - An integrator element based on the Euler method.

    properties (Constant)
        Name = "Forward Euler-method";
    end
    methods (Access = protected)
        function [this] = IntegrateTransform(this,transform,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
%             p0 = pose.GetWorldPosition();
%             q0 = pose.GetWorldRotation();

            p0 = transform.Inertial.Position;
            q0 = transform.Inertial.Rotation;

%             if transform.Entity.Name == "Box 2"
%                 
%             end

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
            transform.Inertial.Position = position;
            transform.Inertial.Rotation = rotation;
        end
    end
end