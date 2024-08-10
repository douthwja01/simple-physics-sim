classdef AnalyticalGlobalConstraintSolver < GlobalConstraintSolver
% The root blass for all constraint solver that use a global/analytical
% approach.

    properties (Constant)
        Name = "A global constraint solver that uses the global-analytical approach.";
    end
    properties

    end

    %% Main
    methods
        function [this] = AnalyticalGlobalConstraintSolver()
            % CONSTRUCTOR - Create an instance of a narrow-phase solver.

            % Initialise the parent
            [this] = this@GlobalConstraintSolver();
        end
    end
    %% Utilties
    methods
        function [this] = Initialise(this,bodies)
            % Initialise the global analytical solver.

        end
        function [this] = Solve(this,dt,constraints)
            % Solve the global set of constraints using the 

        end
    end

end