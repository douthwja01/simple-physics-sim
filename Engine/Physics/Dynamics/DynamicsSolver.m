classdef (Abstract) DynamicsSolver < SimElement
    % The base class for all 'dynamics' solution finders and tools.

    methods (Abstract)
        % Compute the dynamics of a set of bodies
        [this] = Compute(this,bodies);
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