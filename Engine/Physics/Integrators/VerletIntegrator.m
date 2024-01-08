classdef VerletIntegrator < Integrator
    %VERLETINTEGRATOR - An integrator element based on the verlet method.

    properties (Constant)
        Name = "Verlet method";
    end

    methods (Access = protected)
        function [this] = IntegrateTransform(this,transform,dt)
            % Update the object using the verlet method.

            % If the prior transform is not populated
            if isempty(transform.PriorPosition)
                transform.PriorPosition = transform.position;
            end

            % Calculate the implied velocity
            transform.Velocity = (transform.position - transform.PriorPosition)/dt;
            % Store the new prior-position
            transform.PriorPosition = transform.position;
            % Update the position
            transform.position = transform.position + transform.Velocity*dt + transform.Acceleration * dt^2;
        end
    end

                % If the object is static, abort
%             if transform.IsStatic
%                 transform.Velocity = zeros(3,1);
%                 transform.Acceleration = zeros(3,1);
%                 return
%             end
%             % Update the physics
%             transform.Velocity = transform.Velocity + transform.Acceleration*dt;
%             transform.position = transform.position + transform.Velocity*dt + (1/2)*transform.Acceleration*dt^2;
        
end

