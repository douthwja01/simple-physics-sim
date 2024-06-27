classdef VerletIntegrator < Integrator
    %VERLETINTEGRATOR - An integrator element based on the verlet method.

    % Verlet quaternion integration? 
    % https://ubm-twvideo01.s3.amazonaws.com/o1/vault/gdc04/slides/using_verlet_integration.pdf


    properties (Constant)
        Name = "Verlet method";
    end
    methods
        function [state] = Solve(this)
            % This function calculates the integral step using the verlet
            % approach.

            error("Not implemented.");
        end
    end

    methods (Static, Access = protected)
        function [objectData] = IntegrateObject(objectData,dt)
            % Update the object using the verlet method.

            p0 = objectData.PreviousSO3.Position;
            q0 = objectData.PreviousSO3.Rotation;
            p = objectData.SO3.Position;
            w = objectData.AngularVelocity;
            a = objectData.LinearAcceleration;
        

            % Assuming angular_velocity is a Quaternion with 0 real part
            half_dt_omega = w * (dt / 2);
            
            wq = VerletIntegrator.QuaternionHamiltonianProduct(q0,[0;half_dt_omega]);

            % q(t + dt/2) = q(t) + (dt/2) * omega * q(t)
            q_half_step = q0 + wq * q0;
            q_half_step = q_half_step/norm(q_half_step);
            
            % q(t + dt) = q_half_step + (dt/2) * omega * q_half_step
            q = q_half_step + wq * q_half_step;
            q = q/norm(q);
            
     

            % Compute
            position = 2*p - p0 + (dt^2)*a;
            rotation = Quaternion(q);

%             dq = Quaternion.Rate(q0,w);
%             rotation = q0 + dq*dt;

            % Dependant properties
            v = (p - p0)/dt;

            % Update the pose
            objectData.SO3 = SO3(position,rotation);
            % Update the velocities
            objectData.LinearVelocity = v;
            objectData.AngularVelocity = w;
        end
        function [q] = QuaternionHamiltonianProduct(q0,q1)
            % Hamilton product
            w = q0.W * q1.W - q0.X * q1.X - q0.Y * q1.Y - q0.Z * q1.Z;
            x = q0.W * q1.X + q0.X * q1.W + q0.Y * q1.Z - q0.Z * q1.Y;
            y = q0.W * q1.Y - q0.X * q1.Z + q0.Y * q1.W + q0.Z * q1.X;
            z = q0.W * q1.Z + q0.X * q1.Y - q0.Y * q1.X + q0.Z * q1.W;
            q = Quaternion(x, y, z, w);
        end
    end        
end

