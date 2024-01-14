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
            v0 = pose.Velocity;


            % Compute the simple Forward-Euler step
            velocity = v0 + pose.Acceleration*dt;
            position = p0 + velocity*dt;

            pose.Velocity = velocity;
            pose.SetWorldPosition(position);
        end
    end
end