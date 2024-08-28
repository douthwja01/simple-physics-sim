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

            % Sanity check
            assert(isa(bodies,"Particle"),"Expecting an array of particles.");

            % Initialise global
            [matrixIndices] = this.InitialiseGlobalProperties(bodies);
        end
        function [this] = Solve(this,dt,constraints)
            % Solve the global set of constraints using the

            % Sanity check
            if ~isempty(constraints)
                return
            end

        end
    end
    methods (Static,Access = protected)
        function [matrixSet] = InitialiseGlobalProperties(bodies)
            % [TESTING] Solver Global Matrix creation
            xStartIndex = 0;
            yStartIndex = 0;
            for i = 1:numel(bodies)

                joints = bodies(i).Entity.Joints;
                if isempty(joints)
                    dof = 6;
                else
                    dof = joints.DegreesOfFreedom;
                end
                xMinIndex = xStartIndex+1;
                xMaxIndex = xStartIndex+dof;
                yMinIndex = yStartIndex+1;
                yMaxIndex = yStartIndex+dof; % To confirm

                objectData = struct( ...
                    "xMin",xMinIndex, ...
                    "xMax",xMaxIndex, ...
                    "yMin",yMinIndex, ...
                    "yMax",yMaxIndex);
                % Pass on limits
                newIndex = xMaxIndex;

                % Extract new indices
                xStartIndex = objectData.xMax;
                yStartIndex = objectData.yMax;
                % Retain
                matrixSet(i,1) = objectData;
            end
        end
    end
end