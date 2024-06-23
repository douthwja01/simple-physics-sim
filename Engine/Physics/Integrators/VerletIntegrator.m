classdef VerletIntegrator < Integrator
    %VERLETINTEGRATOR - An integrator element based on the verlet method.

    % Verlet quaternion integration? 
    % https://ubm-twvideo01.s3.amazonaws.com/o1/vault/gdc04/slides/using_verlet_integration.pdf


    properties (Constant)
        Name = "Verlet method";
    end

    methods (Static, Access = protected)
        function [objectData] = IntegrateObject(objectData,dt)
            % Update the object using the verlet method.

            p0 = objectData.PreviousSO3.Position;
            q0 = objectData.PreviousSO3.Rotation;
            p = objectData.SO3.Position;

            w = objectData.AngularVelocity;
            a = objectData.LinearAcceleration;
        
            % Compute
            position = 2*p - p0 + (dt^2)*a;
            dq = Quaternion.Rate(q0,w);
            rotation = q0 + dq*dt;

            % Dependant properties
            v = (p - p0)/dt;

            % Update the pose
            objectData.SO3 = SO3(position,rotation);
            % Update the velocities
            objectData.LinearVelocity = v;
            objectData.AngularVelocity = w;
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

