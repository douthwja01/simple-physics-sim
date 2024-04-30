classdef FeatherstoneDynamics < DynamicsSolver
    % This class implements the Featherstone method for resolving 
    % the dynamic properties of a physics simulation.

    properties (Constant)
        Name = "Featherstone Dynamic solver.";
    end

    % Interfaces
    methods (Access = protected)
        function [this] = ResolveVelocities(this,transforms)
            % This function computes the velocities of all the provided
            % transforms.
        end
        function[this] = ResolveAccelerations(this,transforms)
            % This function computes the accelerations of all the provided
            % transforms.
        end
        function [this] = ResolveForces(this,transforms)
            % This function computes the forces of all the provided
            % transforms.
        end
    end  
end