
classdef (Abstract) Solver < matlab.mixin.Heterogeneous & handle
    % The basic solver structure
    
    methods (Abstract)
        % Solve the collisions
        [this] = Solve(collisions,dt);
    end
end