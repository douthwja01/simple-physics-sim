classdef OctreePoint < handle
    % The oct-point is a data class that exposes the connection between a
    % points position in an octree and its parent/attached structure to
    % which it belongs outside the octree.
    
    properties
        Position = zeros(3,1);  % Spacial location
        Cid = uint32.empty;      % Identity code
    end
    
    methods
        % Constructor
        function [this] = OctreePoint(point,cid)
            % OCTPOINT - Construct an instance of the Octpoint class from a
            % position in the octree and its attached reference.

            this.Position = point;
            this.Cid = cid;
        end
        % Get/sets
        function set.Position(this,p)
            assert(IsColumn(p,3),"Expecting a valid cartesian point [3x1].");
            this.Position = p;
        end
        function set.Cid(this,cid)
            assert(isa(cid,"uint32"),"Expecting a 'uint32' cid code.");
            this.Cid = cid;
        end
        % Support functions
        function [h] = Draw(this,container,colour)
            % Plot the octree-point to a graphical container.
            if nargin < 2
                container = gca;
            end
            if nargin < 3
                colour = "k";
            end
            % Plot the position coordinate
            pi = this.Position;
            h = plot3(container,pi(1),pi(2),pi(3),colour);
            set(h, ...
                "MarkerSize",10, ...
                "Marker","^", ...
                "MarkerFaceColor",colour, ...
                "MarkerEdgeColor","k");
        end
    end
end

