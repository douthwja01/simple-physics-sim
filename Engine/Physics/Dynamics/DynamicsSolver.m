classdef (Abstract) DynamicsSolver < SimElement
    % The base class for all 'dynamics' solution finders and tools.

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

    % Internals
    methods (Abstract, Access = protected)
        % Compute the velocities of the transforms
        [this] = ResolveVelocities(this,transforms);
        % Compute the accelerations of the transforms
        [this] = ResolveAccelerations(this,transforms);
        % Compute the forces of the transforms
        [this] = ResolveForces(this,transforms);
    end
end