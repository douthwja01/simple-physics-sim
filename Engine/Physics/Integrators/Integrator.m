classdef (Abstract) Integrator < Module
    % INTEGRATOR - The base definition for modules integrating the state of
    % the simulator.
    
    methods
        function [this] = Integrate(this,state,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            % Sanity check
            assert(isa(state,"WorldState"),"Expecting a valid world-state.");

            for i = 1:state.NumberOfObjects
                objectData = state.Objects(i);

                % Record the last pose
                objectData.PreviousSO3 = objectData.SO3;

                % Is static, do not integrate
                if objectData.IsStatic
                    objectData.LinearVelocity = zeros(3,1);
                    objectData.AngularVelocity = zeros(3,1);
                    objectData.LinearAcceleration = zeros(3,1);
                    objectData.AngularAcceleration = zeros(3,1);
                else
                    objectData = this.IntegrateObject(objectData,dt);
                end
                % Update the structure
                state.Objects(i) = objectData;
            end
        end
    end
   
    %% Internals
    methods (Abstract, Static, Access = protected)
        [objectData] = IntegrateObject(objectData,dt);
    end
end