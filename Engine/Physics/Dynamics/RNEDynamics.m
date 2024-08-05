classdef RNEDynamics < DynamicsModule
    % This class implements the recursive Newton-Euler method for resolving
    % the dynamic properties of a physics simulation.

    properties (Constant)
        Name = "Recursive-Newton Euler (RNE) Dynamic solver.";
    end

    %% Internal
    methods (Access = protected)
        function [this] = ComputeDynamics(this,bodies,dt)
            % Compute the motion of the provided bodies utilitising the
            % given approach.

            for i = 1:numel(bodies)
                body_i = bodies(i);

                % Get the transform data
                tf_i = body_i.Transform;
                orientation = tf_i.GetOrientationParentSpace();
                position = tf_i.GetPositionParentSpace();

                v_i = body_i.LinearVelocity;
                w_i = body_i.AngularVelocity;

%                 orientation = QuatAddScaled(orientation, m_angularVelocity, timeStep);
                dQ = Quaternion.Rate(orientation,w_i);

                orientation = orientation + dQ*dt;
                position = position + v_i*dt;
                
                % Assign zeroth order states
                tf_i.SetOrientation(orientation);
                tf_i.SetPosition(position);
            
                % Calculate the local velocities & accelerations
                dv_i = body_i.LinearAcceleration;
                dv_i = dv_i + body_i.InverseMass*body_i.forceAccumulator;
                dw_i = body_i.InverseInertia*body_i.torqueAccumulator;
            
                v_i = v_i + body_i.linearImpulseAccumulator*body_i.InverseMass;
                w_i = w_i + body_i.angularImpulseAccumulator*body_i.InverseMass;
            
                v_i = v_i + dv_i * dt;
                w_i = w_i + dw_i * dt;
            
                w_i = w_i * body_i.angularDamping^dt;
                v_i = v_i * body_i.linearDamping^dt;

                % Assign the new properties
                body_i.LinearVelocity = v_i;
                body_i.AngularVelocity = w_i;
                body_i.LinearAcceleration = dv_i;
                body_i.AngularAcceleration = dw_i;
            end
        end
    end
end