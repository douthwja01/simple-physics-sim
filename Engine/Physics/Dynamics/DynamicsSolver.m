classdef (Abstract) DynamicsSolver < Module
    % The base class for all 'dynamics' solution finders and tools.

    % Typically
    % 0.  Clear prior step data.
    %
    % 1.  Calculates forces: loop over all force objects, allowing each to 
    % add forces to the particles it influences.
    % 
    % 2.  Calculate constraint forces: on completion of the previous step, 
    % each particle's force accumulator contains the total force on that 
    % particle. In this step, the global equation 11 is setup and solved, 
    % yielding a constraint foce on each particle, which is added into the 
    % applied force.
    % 
    % 3.  Calculate the derivative: Divide force by mass to get
    % acceleration, and gather the dervivatives into a global
    % vector for the solver (integrator?).

%             this.ComputeForces();
%             this.ComputeContraintForces();
%             this.ComputeAccelerations();  
         
    methods
        function [this] = ComputeDynamics(this,bodies)
            % This function is the entry point for the dynamics computation
            % approach.

            % Clears all the dynamic properties/accumulators ahead of
            % recalculation for this frame.
            for i = 1:numel(bodies)
                bodies(i).ClearAccumulators();
            end

            % Compute Dynamics routine (may be different)
            this.TopLevelRoutine(bodies);
        end
    end

    %% Internals
    methods (Abstract, Access = protected)
        % Compute the dynamics of a set of bodies
        [this] = TopLevelRoutine(this,bodies);
        % Compute the velocities of the transforms
        [this] = ComputeVelocities(this,transforms);
        % Compute the accelerations of the transforms
        [this] = ComputeAccelerations(this,transforms);
        % Compute the forces of the transforms
        [this] = ComputeForces(this,transforms);
    end
end