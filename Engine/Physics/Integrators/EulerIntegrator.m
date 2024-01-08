classdef EulerIntegrator < Integrator
    %EULERINTEGRATOR - An integrator element based on the Euler method.

    properties (Constant)
        Name = "Forward Euler-method";
    end
    methods (Access = protected)
        function [this] = IntegrateTransform(this,transform,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            % Compute the simple Forward-Euler step
            transform.Velocity = transform.Velocity + transform.Acceleration*dt;
            transform.position = transform.position + transform.Velocity*dt;
        end
    end
end