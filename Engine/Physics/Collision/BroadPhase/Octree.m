classdef Octree < handle
    % OCTREE - This class defines an oct-tree hierarchical data structure.
    % While developed for solving 3D collision problems, the implementation
    % is generic and can be executed against similar problems.
    
    properties
        Root = OctreeNode.empty;
    end
    
    methods
        function [this] = Octree(boundary,nodeCapacity)
            % Constructor
            
            % Random 100 unitary cube
            if nargin < 1
                boundary = AABB(zeros(3,1),[-50;50],[-50;50],[-50;50]);
            end
            
            if nargin < 2
                nodeCapacity = 4;
            end

            assert(isa(boundary,"AABB"),"Expecting an AABB boundary definition.");
            assert(isscalar(nodeCapacity),"Expecting a scalar node (point) capacity.");
            
            % Create the root
            this.Root = OctreeNode(boundary,nodeCapacity);
        end
        % Main methods
        function [data] = Query(this,queryBounds)
            % Allow the octree to be queried with an boundary to find
            % points within it.

            % Sanity check
            assert(isa(queryBounds,"AABB"),"Expecting a valid query boundary.");
            % Recursively seearch for intersections
            data = this.Root.Query(queryBounds);
        end
        function [flag] = Insert(this,octPoint)
            % This function will insert a simple octPoint into the tree
            % structure.

            % Insert a simple point into the octree.
            flag = this.Root.Insert(octPoint);
        end
    end
    methods
        % Support methods
        function [nodes] = ListNodes(this)
            % This function returns a flattened handle to all nodes in the
            % Octree structure (for debugging).

            % Call find child on the root
            nodes = this.Root.ListChildren();
        end
        function [n] = GetNumberOfNodes(this)
            n = this.Root.GetNumberOfChildren();
        end
        % Drawing methods
        function [h] = DrawNodes(this,container,colour,onlyifPopulated)
            % Draw the complete set of Octree-nodes within the Octree.
            
            % Input parsing
            if nargin < 2
                container = gca;
            end
            if nargin < 3
                colour = 'c';
            end
            if nargin < 4
                onlyifPopulated = false;
            end
            % Draw all nodes within the tree
            h = this.Root.Draw(container,colour,onlyifPopulated);
        end
        function [h] = DrawPoints(this,container,colour)
            % Draw all Octree-points withing the octree.
            
            % Input parsing
            if nargin < 2
                container = gca;
            end
            if nargin < 3
                colour = 'k';
            end
            % As the root to draw its points
            h = this.Root.DrawPoints(container,colour);
        end
    end
end

