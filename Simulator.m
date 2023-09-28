
classdef Simulator < handle

    properties
        TimeDelta = 0.01;
        EnableSubStepping = true;
        SubSteps = 5;
        g = [0;0;-9.81];
        % Contents
        FixedObjects = SimObject.empty;
        Objects = SimObject.empty;
        Links = SimLink.empty;
    end
    properties (Access = private)
        IsStopped = false;
    end

    methods
        function [this] = Simulate(this,duration)
            % This function executes the simulation sequence

            % Sanity check
            assert(isscalar(duration) && duration > 0,"Expecting a scalar duration greater than zero.");

            [ax] = this.InitialiseGraphics();

            if this.EnableSubStepping
                subTimeDelta = this.TimeDelta/this.SubSteps;
            end

            time = 0;
            while (time < duration && ~this.IsStopped)
                fprintf("[t=%.2fs] Stepping.\n",time);

                % Step (or substep) the world
                if this.EnableSubStepping
                    for s = 1:this.SubSteps
                        this.Step(subTimeDelta);
                    end
                else
                    this.Step(this.TimeDelta);
                end
                % Update the visual representation
                this.UpdateGraphics(ax);
                % Integrate the time
                time = time + this.TimeDelta;
            end
        end
        % Add/remove the objects
        function [this] = AddObject(this,newObject)
            assert(isa(newObject,"SimObject"),"Expecting a valid SimObject.");
            this.Objects = vertcat(this.Objects,newObject);
        end
        function [this] = DeleteObject(this,index)
            % Temporary index
            vec = 1:1:numel(this.Objects);
            % Remove the object 
            this.Objects = this.Objects(vec ~= index);
        end
        % Add fixed object
%         function [this] = AddFixedObject(this,fixedObject)
%             assert(isa(fixedObject,"SimObject"),"Expecting a valid SimObject.");
%             this.FixedObjects = vertcat(this.FixedObjects,newObject);
%         end                
        % Add links
        function [this] = AddLink(this,objectA,objectB)
            assert(isa(objectA,"SimObject"),"Expecting a first valid SimObject.");
            assert(isa(objectB,"SimObject"),"Expecting a second valid SimObject.");
            % Create a new link object
            newLink = Link(objectA,objectB);
            this.Links = vertcat(this.Links,newLink);
        end
%         function [this] = AddFixedObject(this,fixed)
% 
%         end
    end

    % Updates
    methods (Access = private)
        function [this] = Step(this,TimeDelta)
            % The step procedure
            this.ApplyGravity();
            this.UpdatePositions(TimeDelta);
            this.ApplyWorldConstraint();
            this.SolveCollisions();
            this.ApplyLinks();
        end
        % Physics
        function [this] = ApplyGravity(this)
            % This function applies gravity to all particles

            for i = 1:numel(this.Objects)
                % Apply gravity
                this.Objects(i).Accelerate(this.g);
            end
        end
        function [this] = UpdatePositions(this,timeDelta)
            % Update the physics properties of the world.

            for i = 1:numel(this.Objects)
                this.Objects(i).Update(timeDelta);
            end
        end
        % Constraints
        function [this] = ApplyLinks(this)
            % This function applys the set of link constraints to the
            % system.
            for i = 1:numel(this.Links)
                this.Links(i).Apply();
            end
        end
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
        function [this] = UpdateGraphics(this,ax)
            % This function updates the graphical handles of all the
            % objects.

            for i = 1:numel(this.Objects)
                this.Objects(i).UpdateGraphics(ax);
                drawnow;
            end
        end
    end

    % Setup
    methods (Access = private)
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

            set(fig,'KeyPressFcn',@(src,evnt)KeyPressCallback(this,src,evnt));
        end
        function KeyPressCallback(this,src,event)
            % This function is called on key press withthe active figure.
            % disp(event.Key);

            if strcmpi(event.Key,"escape")
                fprintf("Simulation Stopped.\n");
                this.IsStopped = true;
            else
                warning("Press 'escape' to stop the simulation.");
            end
        end
    end
end