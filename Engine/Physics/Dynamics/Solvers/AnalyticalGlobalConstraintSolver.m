classdef AnalyticalGlobalConstraintSolver < GlobalConstraintSolver
% The root blass for all constraint solver that use a global/analytical
% approach.

    properties (Constant)
        Name = "A global constraint solver that uses the global-analytical approach.";
    end

    methods
        function [this] = AnalyticalGlobalConstraintSolver()
            % CONSTRUCTOR - Create an instance of a narrow-phase solver.

            % Initialise the parent
            [this] = this@GlobalConstraintSolver();
        end

        function [this] = Solve(this,dt,constraints)
            % Solve the global set of constraints using the 


        end
    end

end