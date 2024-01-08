
classdef OctreeNode < handle

    properties
        Capacity = 2;
        Children = OctreeNode.empty;
        Points = OctreePoint.empty;
    end
    properties (SetAccess = private)
        Boundary = AABB.empty();
    end
    properties
        NumberOfPoints;
    end

    methods
        function [this] = OctreeNode(boundary,capacity)
            % The constructor for an octree-node        
            assert(isa(boundary,"AABB"),"Expecting a valid AABB boundary.");
            this.Boundary = boundary;
            this.Capacity = capacity;
        end
        function set.Points(this,p)
            assert(isa(p,"OctreePoint"),"Expecting a valid Octree-point.");
            this.Points = p;
        end
        function [n] = get.NumberOfPoints(this)
            n = length(this.Points);
        end
        % Main Methods
        function [data] = Query(this,region)
            % This function takes a AABB query and determines the complete
            % set of points that exist within it.

            data = {};
            
            % If the query doesn't intersect this region, abort. 
            if ~this.Boundary.Intersects(region)
                return;
            end

            % Check all points in the node against the boundary.
            for i = 1:this.NumberOfPoints
                octPoint = this.Points(i);

                if ~region.Contains(octPoint.Position)
                    continue
                end
                % Append the contained point
                data = vertcat(data,octPoint);
            end

            % If it has not divided
            if numel(this.Children) == 0
                return
            end

            for i = 1:numel(this.Children)
                c_data = this.Children(i).Query(region);
            end
        end
        function [flag] = InsertPoint(this,octPoint)
            % Allow the insertion of a given point into the octreeNode.
 
            % Sanity check
            assert(isa(octPoint,"OctreePoint"),"Expecting a valid octree point.");

            % 1. Check if the point is applicable to this node at all.
            flag = false;
            if ~this.Boundary.Contains(octPoint.Position)
                return
            end
            
            % 2. Check if this node is at capacity
            if this.Capacity > this.NumberOfPoints
                % Add the point to the array
                this.Points = vertcat(this.Points,octPoint); 
                flag = true;
                return;
            end

            % If we are at capacity, we need to divide
            if numel(this.Children) == 0
                this.SubDivide();
            end

            for i=1:numel(this.Children)
                % Attempt insertion in each child-node
                if this.Children(i).InsertPoint(octPoint)
                    flag = true;
                    return
                end
            end
        end
        % Support functions
        function [n] = GetNumberOfChildren(this)
            % Get the number of children under this node
            n = 1; % Include itself
            for i = 1:numel(this.Children)
                ni = this.Children(i).GetNumberOfChildren;
                n = n + ni;
            end
        end
        function [h] = Draw(this,container,colour,onlyifPopulated)
            % Allow drawing of the octree node.

            if nargin < 2
                container = gca;
            end
            if nargin < 3
                colour = "r";
            end
            if nargin < 4
                onlyifPopulated = false;
            end

            h = [];
            if this.NumberOfPoints < 1 && onlyifPopulated
                return
            end

            % Plot the boundary from its extents
            mesh = MeshGenerator.CuboidFromExtents( ...
                this.Boundary.Min, ...
                this.Boundary.Max);
            h = mesh.Draw(container,colour);
            set(h,"FaceAlpha",0.1);

            % Ask all children to draw themselves
            for i = 1:numel(this.Children)
                this.Children(i).Draw(container,colour,onlyifPopulated);
            end
        end
        function [h] = DrawPoints(this,container,colour)
            % Draw all the points within the tree
            if nargin < 2
                container = gca;
            end
            if nargin < 3
                colour = "r";
            end

            % Draw all my points
            h = [];
            for i = 1:this.NumberOfPoints
                this.Points(i).Draw(container,colour);
            end

            % Ask children to draw all their points
            for i = 1:numel(this.Children)
                this.Children(i).DrawPoints(container,colour);
%                 h = vertcat(h,ch);
            end
        end
    end
    methods (Access = private)
        function [this] = SubDivide(this)
            % This function generates a set of child nodes within the
            % current node.

            % Calculate new ranges, zero'd around the child position
            quaterRanges = this.Boundary.Span/4;
            nodeCenter = this.Boundary.GetMidPoint();

            unitaryCuboid = [
                1, 1, 1;
                -1, 1, 1;
                 1,-1, 1;
                -1,-1, 1;
                 1, 1,-1;
                -1, 1,-1;
                 1,-1,-1;
                -1,-1,-1];

            % Child centroids
            childCenters = unitaryCuboid.*quaterRanges' + nodeCenter';

            for i = 1:8
                % Position the child centers
                childCenter_i = childCenters(i,:)';
                % Create the new sub-boundary
                childBoundary = AABB( ...
                    [-1,1]*quaterRanges(1) + childCenter_i(1), ...
                    [-1,1]*quaterRanges(2) + childCenter_i(2), ...
                    [-1,1]*quaterRanges(3) + childCenter_i(3));
                % Assign the new child
                this.Children(i) = OctreeNode(childBoundary,this.Capacity);
            end
        end
    end
end