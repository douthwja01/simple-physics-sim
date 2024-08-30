classdef NieveDynamics < DynamicsModule

    properties (Constant)
        Name = "A Nieve dynamics approach to test dynamics module abstraction.";
    end
    properties
        Gravity = [0;0;-9.81];
    end
    methods
        function [this] = Initialise(this,world)
            % Do nothing by default (may need solver initialisation)

            % Sanity check
            assert(isa(world,"PhysicsWorld"),"Expecting a valid PhysicsWorld reference.");
            % Property assignment
            this.Gravity = world.Gravity;
        end
        function [this] = Update(this,dt,bodies)
            % This function is the entry point for the dynamics computation
            % approach.

            % Compute Dynamics routine (may be different)
            this.ComputeDynamics(dt,bodies);
            
%             % Clears all the dynamic properties/accumulators for the next
%             % frame.
%             for i = 1:numel(bodies)
%                 bodies(i).ClearAccumulators();
%             end
        end
    end

    %% Internals
    methods (Access = protected)
        % Compute the dynamics of a set of bodies
        function [this] = ComputeDynamics(this,dt,bodies)
            % This function applies gravity to all particles

            % Update rigidbodies (accelerations)
            for i = 1:numel(bodies)
                % If this body is not effected by gravity
                if ~bodies(i).IsDynamic
                    continue;
                end
                % Apply gravity
                bodies(i).LinearAcceleration = this.Gravity;
            end
        end
    end
end