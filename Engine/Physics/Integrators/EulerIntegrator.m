classdef EulerIntegrator < Integrator
    %EULERINTEGRATOR - An integrator element based on the Euler method.

    properties (Constant)
        Name = "Forward Euler-method";
    end
    methods
        function [state] = Solve(this)
            % This function calculates the integration step using the
            % defined approach.

            % Copy the structure
            state = this.InitialState;

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
                    objectData = this.IntegrateBody(objectData,this.TimeDelta);
                end
                % Update the structure
                state.Objects(i) = objectData;
            end
        end
    end
end