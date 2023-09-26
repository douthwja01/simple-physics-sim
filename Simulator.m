
classdef Simulator < handle

    properties 
        Objects = SimObject.empty;
        dt = 0.01;
        g = [0;0;-9.81];
    end

    methods
        function [this] = Simulate(this,duration)
            
            % Sanity check
            assert(isscalar(duration) && duration > 0,"Expecting a scalar duration greater than zero.");
            
            [ax] = this.InitialiseGraphics();

            time = 0;
            while (time < duration)

                fprintf("[t=%ds] Stepping.\n",time);

                this.ApplyGravity();

                this.UpdatePhysics(this.dt);

                this.ApplyConstraints();

                this.UpdateGraphics(ax);
                
                % Integrate the time
                time = time + this.dt;
            end
        end
    end

    methods (Access = private)
        % Physics
        function [this] = ApplyGravity(this)
            for i = 1:numel(this.Objects)
                % Apply gravity
                this.Objects(i).ApplyGravity(this.g);
            end
        end
        function [this] = UpdatePhysics(this,dt)
            % Update the physics properties of the world.

            for i = 1:numel(this.Objects)
                this.Objects(i).Update(dt);
            end
        end
        % Constraints
        function [this] = ApplyConstraints(this)
            for i = 1:numel(this.Objects)
                object_i = this.Objects(i);
                
                % If the object is below the ground plane, stop.
%                 if object_i.p(3) < 0
%                     object_i.p(3) = 0; 
%                 end

                vector = (object_i.p - worldCenter);
                if norm(vector) > 5
                    object_i.p = 
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
            xlabel(ax,"X (m)");
            ylabel(ax,"Y (m)");
            zlabel(ax,"Z (m)");
            xlim(ax,[-axisLimits,axisLimits]);
            ylim(ax,[-axisLimits,axisLimits]);
            zlim(ax,[0,axisLimits]);
            axis equal;
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