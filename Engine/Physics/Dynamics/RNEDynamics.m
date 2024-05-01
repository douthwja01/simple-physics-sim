classdef RNEDynamics < DynamicsSolver
    % This class implements the recursive Newton-Euler method for resolving
    % the dynamic properties of a physics simulation.

    properties (Constant)
        Name = "Recursive-Newton Euler (RNE) Dynamic solver.";
    end

    % Interface
    methods
        function [this] = Compute(this,bodies)
            % Compute the motion of the provided bodies utilitising the
            % given approach.

            % Resolve velocities
            this.ResolveVelocities(bodies);
            % Resolve velocities
            this.ResolveAccelerations(bodies);
            % Resolve velocities
            this.ResolveForces(bodies);
        end
    end
    % Internal
    methods (Access = protected)
        function [this] = ResolveVelocities(this,transforms)
            % This function computes the velocities of all the provided
            % transforms.

            % We can assume that all bodies are non-static.

            for i = 1:numel(transforms)
                transform_i = transforms(i);
                % Get the joint
                joint = transform_i.Entity.Joints;
                if isempty(joint)
                    continue;
                end

                
            end
        end
        function[this] = ResolveAccelerations(this,transforms)
            % This function computes the accelerations of all the provided
            % transforms.

            for i = 1:numel(transforms)
                transform_i = transforms(i);

            end
        end
        function [this] = ResolveForces(this,transforms)
            % This function computes the forces of all the provided
            % transforms.

            for i = 1:numel(transforms)
                transform_i = transforms(i);

            end
        end
    end
end