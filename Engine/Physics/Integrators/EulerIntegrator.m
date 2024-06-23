classdef EulerIntegrator < Integrator
    %EULERINTEGRATOR - An integrator element based on the Euler method.

    properties (Constant)
        Name = "Forward Euler-method";
    end
    methods (Static, Access = protected)
        function [objectData] = IntegrateObject(objectData,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            p0 = objectData.SO3.Position;
            q0 = objectData.SO3.Rotation;

            v0 = objectData.LinearVelocity;
            w0 = objectData.AngularVelocity;

            % Compute the simple Forward-Euler step
            v = v0 + objectData.LinearAcceleration*dt;
            w = w0 + objectData.AngularAcceleration*dt;
            
            % Compute
            dq = Quaternion.Rate(q0,w);

            % Euler step
            rotation = q0 + dq*dt;
            position = p0 + v*dt;

            % Update the pose
            objectData.SO3 = SO3(position,rotation);
            % Update the velocities
            objectData.LinearVelocity = v;
            objectData.AngularVelocity = w;
        end
    end
end