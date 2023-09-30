
classdef CollisionPoints
    % A simple class describing a collision violation.
    properties
        A = zeros(3,1);         % Furtherest point of A into B
        B = zeros(3,1);         % Furtherest point of B into A
        Normal = zeros(3,1);    % B -A normalised vector
        Depth = 0;              % Length of Normal
        HasCollision = false;   % Collision flag
    end

    methods
        function [this] = CollisionPoints(a,b,n,d)
            % Constructor for the collision points object.
            this.A = a;
            this.B = b;
            this.Normal = n;
            this.Depth = d;
        end
    end
end