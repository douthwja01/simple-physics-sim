
classdef Simulator < handle

    properties 
        EnableSubStepping = true;
        SubSteps = 5;


        Objects = SimObject.empty;
        dt = 0.01;
        g = [0;0;-9.81];
    end

    methods
        function [this] = Simulate(this,duration)
            
            % Sanity check
            assert(isscalar(duration) && duration > 0,"Expecting a scalar duration greater than zero.");
            
            [ax] = this.InitialiseGraphics();

            sub_dt = this.dt/this.SubSteps;

            time = 0;
            while (time < duration)
                fprintf("[t=%.2fs] Stepping.\n",time);

                if this.EnableSubStepping
                    for s = 1:this.SubSteps
                        this.Step(sub_dt);
                    end
                else
                    this.Step(this.dt);
                end
                % Update the visual representation
                this.UpdateGraphics(ax);
                % Integrate the time
                time = time + this.dt;
            end
        end
    end

    methods (Access = private)
        function [this] = Step(this,dt)
            % The step procedure
            this.ApplyGravity();
            this.UpdatePositions(dt);
            this.ApplyWorldConstraint();
            this.SolveCollisions();
          
        end
        % Physics
        function [this] = ApplyGravity(this)
            % This function applies gravity to all particles
            
            for i = 1:numel(this.Objects)
                % Apply gravity
                this.Objects(i).ApplyGravity(this.g);
            end
        end
        function [this] = UpdatePositions(this,dt)
            % Update the physics properties of the world.

            for i = 1:numel(this.Objects)
                this.Objects(i).Update(dt);
            end
        end
        % Constraints
        function [this] = SolveCollisions(this)
            % This function solves the inter-particle collisions
            for i = 1:numel(this.Objects)
                object_i = this.Objects(i);

                for j = 1:numel(this.Objects)
                    object_j = this.Objects(j);

                    % Check if assessing self-collision
                    if (object_i == object_j)
                        continue
                    end

                    collisionAxis = object_i.Position - object_j.Position;
                    distance = norm(collisionAxis);
                    radialSum = object_i.Radius + object_j.Radius;
                    if distance < radialSum
                        unitCollisionAxis = collisionAxis/distance;
                        delta = radialSum - distance;
                        object_i.Position = object_i.Position + 0.5*delta*unitCollisionAxis;
                        object_j.Position = object_j.Position - 0.5*delta*unitCollisionAxis;
                    end

                end
            end

        end
        function [this] = ApplyWorldConstraint(this)
            % This function applies the world-constraint of a fixed sphere.

            globeCenter = [0;0;10];
            globeRadius = 10;

            for i = 1:numel(this.Objects)
                object_i = this.Objects(i);
                
                v = object_i.Position - globeCenter;
                distance = norm(v);
                constraintDistance = globeRadius - object_i.Radius;
                if distance > constraintDistance
                    unit_v = v/distance;
                    delta = globeRadius - object_i.Radius;
                    object_i.Position = globeCenter + delta*unit_v;
                end
            end
        end
        % Graphics 
        function [ax,this] = InitialiseGraphics(this)
            % Draw the state of the world
           
            axisLimits = 10;

            fig = figure("Name","Simulation");
            ax = axes(fig);
            hold on;
            set(ax,"View",[35,35])
            grid on;
            box on;
            axis equal;
            xlabel(ax,"X (m)");
            ylabel(ax,"Y (m)");
            zlabel(ax,"Z (m)");
            xlim(ax,[-axisLimits,axisLimits]);
            ylim(ax,[-axisLimits,axisLimits]);
            zlim(ax,[0,axisLimits]);
        end
        function [this] = UpdateGraphics(this,ax)
            % This function updates the graphical handles of all the
            % objects.

            for i = 1:numel(this.Objects)
                this.Objects(i).UpdateGraphics(ax);
                drawnow;
            end
        end
    end
end