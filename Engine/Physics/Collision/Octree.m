classdef Octree < handle
    %OCTTREE Summary of this class goes here
    %   Detailed explanation goes here
    
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


            data = [];
        end
        function [flag] = InsertCollider(this,collider)
            % Insert a complete collider into the root node.
            
        end
        function [flag] = InsertPoint(this,point)
            % Insert the point into the root node
            flag = this.Root.InsertPoint(point);
        end
        function [n] = GetNumberOfNodes(this)
            n = this.Root.GetNumberOfChildren();
        end
        function [h] = Draw(this,container)
            % Draw the complete Octree.

            if (nargin < 2)
                container = gca;
            end
            h = this.Root.Draw(container);
        end
    end

end

