classdef OctreePoint < handle
    % The oct-point is a data class that exposes the connection between a
    % points position in an octree and its parent/attached structure to
    % which it belongs outside the octree.
    
    properties
        Position = zeros(3,1);
        Reference = [];
    end
    
    methods
        function [this] = OctreePoint(point,reference)
            % OCTPOINT - Construct an instance of the Octpoint class from a
            % position in the octree and its attached reference.

            this.Position = point;
            this.Reference = reference;
        end
        function set.Position(this,p)
            assert(IsColumn(p,3),"Expecting a valid cartesian point [3x1].");
            this.Position = p;
        end
    end
end

