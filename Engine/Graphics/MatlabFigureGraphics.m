classdef MatlabFigureGraphics < GraphicsModule
    % A basic graphics element designed to output visuals to the matlab
    % figure api.

    properties (Constant)
        Name = "Matlab figure graphical output.";
    end
    properties
        Axes = matlab.graphics.GraphicsPlaceholder.empty;
    end
    
    methods
        function [this] = Initialise(this,worldSize)
            % Draw the state of the world

            fig = figure("Name","Simulation");
            this.Axes = axes(fig);
            hold on;
            set(this.Axes,"View",[35,35])
            grid on;
            box on;
            axis equal;
            xlabel(this.Axes,"X (m)");
            ylabel(this.Axes,"Y (m)");
            zlabel(this.Axes,"Z (m)");
            axisLimits = worldSize/2;
            xlim(this.Axes,[-axisLimits,axisLimits]);
            ylim(this.Axes,[-axisLimits,axisLimits]);
            zlim(this.Axes,[0,worldSize]);
            % Register for key presses
            set(fig,'KeyPressFcn',@(src,evnt)OnKeyPressCallback(this,src,evnt));
        end
        % Update the representation
        function [this] = Update(this)
            % This function updates the graphical handles of all the
            % objects.

            for i = 1:numel(this.Renderers)
                % Get the renderer frome the entity
                this.Renderers(i).Update(this.Axes);
            end
            drawnow;
        end
    end
end