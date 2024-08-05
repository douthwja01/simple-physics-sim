classdef WorldState < handle
    % World-State represents the state of the world in an integratable
    % format for solvers of different types. 
    
    properties
        NumberOfObjects = 0;
        Objects;
    end
    
    methods
        function [this] = WorldState(numberOfbodies)
            %WORLDSTATE Construct an instance of this class
            %   Detailed explanation goes here
            
            [this] = this.Resize(numberOfbodies);
        end
        
        function [this] = Resize(this,n)
            % Resize the state object for s given number of objects.

            % Sanity check
            assert(isscalar(n),"Expecting a scalar number of objects.");

            this.NumberOfObjects = n;
            this.Objects = WorldState.GetStateStructure();
            % Create the object structures
            for i = 1:n
                this.Objects(i) = WorldState.GetStateStructure();
            end
        end

    end
    methods (Static, Access = private)
        function [data] = GetStateStructure()
            % The state structre representing the integratable states of 3D 
            % object.

            data = struct( ...
                "IsStatic",false,...
                "PreviousSO3",SO3.Identity,...
                "SO3",SO3.Identity,...
                "LinearVelocity",zeros(3,1),...
                "LinearAcceleration",zeros(3,1),...
                "LinearMomentum",zeros(3,1),...
                "AngularVelocity",zeros(3,1),...
                "AngularAcceleration",zeros(3,1),...
                "AngularMomentum",zeros(3,1));
        end
    end
end

