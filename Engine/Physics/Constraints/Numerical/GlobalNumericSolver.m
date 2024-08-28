classdef GlobalNumericSolver < GlobalConstraintSolver
% The root blass for all constraint solver that use a global/analytical
% approach.

    properties (Constant)
        Name = "A nieve global solver implementation that solves constraints numerically.";
    end
    properties
        ChildSolvers = NumericConstraintSolver.empty;
    end
    %% Main
    methods
        function [this] = GlobalNumericSolver()
            % CONSTRUCTOR - Create an instance of a narrow-phase solver.

            % Initialise the parent
            [this] = this@GlobalConstraintSolver();
        end
    end
    %% Utilties
    methods
        function [this] = Initialise(this,bodies)
            % Initialise the global numeric solver.

            % Sanity check
            assert(isa(bodies,"Particle"),"Expecting an array of particles/rigidbodies.");

            % Solver
            this.ChildSolvers = [ImpulseSolver()];%,PositionSolver()];
        end
        function [this] = Solve(this,dt,constraints)
            % Solve the global set of constraints using the nested child
            % solvers to numerically resolve the constraints.

            % Sanity check
            if ~isempty(constraints)
                return
            end

            for i = 1:numel(this.ChildSolvers)
                % Solve the collisions
                this.ChildSolvers(i).Solve(constraints,dt);
            end
        end
    end
end