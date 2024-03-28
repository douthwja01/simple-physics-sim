
classdef OctreeNode < handle

    properties
        Capacity = 2;
        Children = OctreeNode.empty;
        Points = OctreePoint.empty;
        NumberOfPoints;
    end
    properties (SetAccess = private)
        HasDivided = false;
        Boundary = AABB.empty();
    end
    properties (Constant)
        SubRegions = 8;
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
        function [results] = Query(this,region)
            % This function takes a AABB query and determines the complete
            % set of points that exist within it.

            results = [];
            
            % If the query doesn't intersect this region, abort. 
            if ~this.Boundary.Intersects(region)
                return;
            end

            % Check all points in the node against the boundary.
            for i = 1:this.NumberOfPoints
                octPoint = this.Points(i);

                if ~region.Contains(this.Points(i).Position)
                    continue
                end

                % Add the point to the results
                results = vertcat(results,octPoint);
            end

            % If it has not divided
            if ~this.HasDivided
                return
            end

            for i = 1:this.SubRegions
                % Query the region against the child
                childResults = this.Children(i).Query(region);
                % Add the point to the results
                if ~isempty(childResults)
                    results = vertcat(results,childResults);
                end
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
            if this.NumberOfPoints < this.Capacity 
                % Add the point to the array
                this.Points = vertcat(this.Points,octPoint); 
                flag = true;
                return;
            end

            % If we are at capacity, we need to divide
            if ~this.HasDivided
                this.SubDivide();
            end
            
            % Try and insert the point in the child node
            for i = 1:this.SubRegions
                % Attempt insertion in each child-node
                if this.Children(i).InsertPoint(octPoint)
                    flag = true;
                    return
                end
            end
        end
        % Support functions
        function [nodes] = ListChildren(this,populatedOnly)
            % Input handling
            if nargin < 2
                populatedOnly = false;
            end
%             if nargin < 2
%                 parentNodes = OctreeNode.empty;
%             end

            % Check if the node is active
            if populatedOnly && numel(this.Points) == 0
                nodes = OctreeNode.empty;
                return 
            end

            % Add itself
            nodes = this;
            
            % If it hasnt divided, return
            if ~this.HasDivided
                return;
            end

            % Add all nodes hierarchically            
            for i = 1:this.SubRegions
                childNodes = this.Children(i).ListChildren(populatedOnly);
                nodes = vertcat(nodes,childNodes);
            end
        end
        function [n] = GetNumberOfChildren(this)
            % Get the number of children under this node
            n = 1; % Include itself

            if ~this.HasDivided
                return;
            end
            for i = 1:this.SubRegions
                n = n + this.Children(i).GetNumberOfChildren;
            end
        end
        % Drawing functions
        function [h] = Draw(this,container,colour,onlyifPopulated)
            % Allow drawing of the octree nodes recursively.

            % Input check
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
            mesh = MeshExtensions.CuboidFromExtents( ...
                this.Boundary.Min, ...
                this.Boundary.Max);
            h = mesh.Draw(container,colour);
            set(h,"FaceAlpha",0.05);

            % Nothing else to draw?
            if ~this.HasDivided
                return;
            end

            % Ask all children to draw themselves
            for i = 1:this.SubRegions
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

            % Nothing else to draw?
            if ~this.HasDivided
                return;
            end

            % Ask children to draw all their points
            for i = 1:this.SubRegions
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

            for i = 1:this.SubRegions
                % Position the child centers
                childCenter_i = childCenters(i,:)';
                % Create the new sub-boundary
                childBoundary = AABB( ...
                    [-1,1]*quaterRanges(1) + childCenter_i(1), ...
                    [-1,1]*quaterRanges(2) + childCenter_i(2), ...
                    [-1,1]*quaterRanges(3) + childCenter_i(3));
                % Assign the new child
                this.Children(i,1) = OctreeNode(childBoundary,this.Capacity);
            end
            % Indicate it has divided
            this.HasDivided = true;
        end
    end
end