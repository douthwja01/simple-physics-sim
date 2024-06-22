classdef (Abstract) Integrator < Module
    % INTEGRATOR - The base definition for modules integrating the state of
    % the simulator.
    
    properties
        InitialState = WorldState.empty;
        TimeDelta = 0;
    end
    methods (Abstract)
        [state] = Solve(this);
    end
    methods
        function [this] = Start(state,dt)
            % This function is called to initialise the integrator for the
            % given state and step time.
            this.TimeDelta = dt;
            this.InitialState = state; 
        end
        function [this] = End(this)
            % This function is called to dispose of the integrator
            this.TimeDelta = 0;
            this.InitialState = WorldState.empty;
        end
    end
    methods (Static, Access = protected)
        function [objectData] = IntegrateBody(objectData,dt)
            % This function computes the euler step for a set of provided
            % bodies/transforms.
            
            p0 = objectData.SO3.Position;
            q0 = objectData.SO3.Rotation;

            v0 = objectData.LinearVelocity;
            w0 = objectData.AngularVelocity;

            % Compute the simple Forward-Euler step
            v = v0 + objectData.LinearAcceleration*dt;
            w = w0 + objectData.AngularAcceleration*dt;
            
            % Compute
            dq = Quaternion.Rate(q0,w);

            % Euler step
            rotation = q0 + dq*dt;
            position = p0 + v*dt;

            % Update the pose
            objectData.SO3 = SO3(position,rotation);
            % Update the velocities
            objectData.LinearVelocity = v;
            objectData.AngularVelocity = w;
        end
    end
end