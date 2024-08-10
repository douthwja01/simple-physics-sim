classdef (Abstract) GlobalConstraintSolver < ConstraintSolver
% The root blass for all constraint solver that use a global/analytical
% approach.

    methods
        function [this] = GlobalConstraintSolver()
            % CONSTRUCTOR - Create an instance of a narrow-phase solver.
        end
    end
    methods (Abstract)
        [this] = Initialise(this,bodies);
    end
end