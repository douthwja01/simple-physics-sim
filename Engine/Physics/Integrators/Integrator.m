classdef (Abstract) Integrator < Module
    % INTEGRATOR - The base definition for modules integrating the state of
    % the simulator.
    
    methods
        function [this] = Integrate(this,bodies,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            for i = 1:numel(bodies)
                body_i = bodies(i);
                tf_i = body_i.Transform;
                % If the object is static, abort
                if tf_i.IsStatic
                    tf_i.Velocity = zeros(3,1);
                    tf_i.Acceleration = zeros(3,1);
                else
                    % Integrate the transform states
                    this.IntegrateTransform(tf_i,dt);
                end
            end
        end
    end
    
    % Private methods 
    methods (Abstract, Access = protected)
        [this] = IntegrateTransform(this,transform,dt);
    end
end