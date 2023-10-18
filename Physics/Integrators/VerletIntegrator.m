classdef VerletIntegrator < Integrator
    %VERLETINTEGRATOR - An integrator element based on the verlet method.
    
    methods
        function [this] = VerletIntegrator(varargin)
            % CONSTRUCTOR - VerletIntegrator
            
            % Integrator object constructor
            %this = this@Integrator(varargin{:});
        end
        
        function [this] = Integrate(this,bodies,dt)
            % Update the object using the verlet method.

            for i = 1:numel(bodies)
                tf_i = bodies(i).Entity.GetElement("Transform");

                % If the object is static, abort
                if tf_i.IsStatic
                    tf_i.Velocity = zeros(3,1);
                    tf_i.Acceleration = zeros(3,1);
                    continue;
                else
                    % Update the physics
                    tf_i.Velocity = tf_i.Velocity + tf_i.Acceleration*dt;
                    tf_i.position = tf_i.position + tf_i.Velocity*dt + (1/2)*tf_i.Acceleration*dt^2;
                end
            end
        end
    end
end

