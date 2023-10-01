
classdef (Abstract) Solver < matlab.mixin.Heterogeneous & handle

    methods (Abstract)
        % Solve the collisions
        [this] = Solve(collisions,dt);
    end
end