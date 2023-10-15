
classdef CollisionPoints
    % A simple class describing a collision violation.
    properties
        A = zeros(3,1);         % Furtherest point of A into B
        B = zeros(3,1);         % Furtherest point of B into A
        Normal = zeros(3,1);    % B -A normalised vector
        Depth = 0;              % The depth of penetration (should be -ve)
        IsColliding = false;    % Collision flag
    end

    methods
        function [this] = CollisionPoints(a,b,n,d,isColliding)
            % Constructor for the collision points object.

            % Sanity check
            assert(nargin == 0 || nargin == 5,"Expecting either 0 or 5 inputs.");
            if nargin == 0
                return;
            end

            this.A = a;
            this.B = b;
            this.Normal = n;
            this.Depth = d;
            this.IsColliding = isColliding;
        end

        function [in] = SwapPoints(this)
            % Swap the points 

            in = this;
            in.A = this.B;
            in.B = this.A;
            in.Normal = - this.Normal;
        end
    end
end