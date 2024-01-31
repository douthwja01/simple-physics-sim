
classdef ContactPoints
    % A simple class describing a collision violation.
    properties
        A = zeros(3,1);         % Furtherest point of A into B
        B = zeros(3,1);         % Furtherest point of B into A
        Normal = zeros(3,1);    % B -A normalised vector
        Depth = 0;              % The depth of penetration (should be -ve)
        IsColliding = false;    % Collision flag
    end

    methods
        function [this] = ContactPoints(a,b,n,d,isColliding)
            % Constructor for the collision points object.

            % Sanity check
            assert(nargin == 0 || nargin == 5,"Expecting either 0 or 5 inputs.");
            if nargin == 0
                return;
            end

            % Ensure numeric consistency
            assert(d > 0,"Expecting a positive depth value")
            % Assign parameters
            this.A = a;
            this.B = b;
            this.Normal = n;
            this.Depth = d;
            this.IsColliding = isColliding;
        end
        function [out] = Swap(this)
            % Swap the points 
            out = ContactPoints();
            out.A = this.B;                     % Set the points of A as B
            out.B = this.A;                     % Set the points of B as A
            out.Normal = -this.Normal;          % Also change the direction
            out.Depth = this.Depth;             % Same scalar distance
            out.IsColliding = this.IsColliding; % Still colliding
        end

        function [h] = Draw(this,container)
            % Draw the collision points to a given container.

            h = [];
            if nargin < 2
                container = gca;
            end
            if isempty(this)
                return;
            end

            markerSize = 3;
            arrowRatio = 0.5;

            % Plot the collision points
            h(1) = plot3(container,this.A(1),this.A(2),this.A(3), ...
                "color","r", ...
                "MarkerSize",markerSize, ...
                "MarkerFaceColor","r", ...
                "Marker","o");
            h(2) = plot3(container,this.B(1),this.B(2),this.B(3), ...
                "color","b", ...
                "MarkerSize",markerSize,...
                "MarkerFaceColor","b",...
                "Marker","o");
            % The seperation arrow
            if this.IsColliding
                arrowColour = "r";
            else
                arrowColour = "b";
            end
            h(3) = mArrow3( ...
                this.A, ...
                this.B, ...
                "color",arrowColour, ...
                "stemWidth",arrowRatio,...
                "tipWidth",0.1*arrowRatio);
        end
    end
end