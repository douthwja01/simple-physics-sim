
classdef OctreeNode < handle

    properties
        Capacity = 4;
        Children = OctreeNode.empty;
        Points = OctreePoint.empty;
    end
    properties (SetAccess = private)
        Boundary = AABB.empty();
        HasDivided = false;
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
            for i = 1:numel(this.Points)
                octPoint = this.Points(i);

                if ~region.Contains(octPoint.Position)
                    continue
                end
                % Append the contained point
                data = vertcat(data,octPoint);
            end

            if ~this.HasDivided
                return
            end

            for i = 1:numel(this.Children)
                c_data = this.Children(i).Query(region);
            end
        end
        function [flag] = InsertPoint(this,octPoint)
            % Allow the insertion of a given point into the octreeNode.
 
            % 1. Check if the point is applicable to this node at all.
            flag = false;
            if ~this.Boundary.Contains(octPoint.Position)
                return
            end
            
            % 2. Check if this node is at capacity
            currentOccupancy = numel(this.Points);
            if this.Capacity > currentOccupancy
                % Add the point to the array
                this.Points = vertcat(this.Points,octPoint); 
                flag = true;
                return;
            end

            % If we are at capacity, we need to divide
            if ~this.HasDivided
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
        function [h] = Draw(this,container)
            % Allow drawing of the octree node.

            if nargin < 2
                container = gca;
            end
            
            % Draw the cuboid
            plot3(container,this.Boundary.Center(1),this.Boundary.Center(2),this.Boundary.Center(3),"ro");
            min = this.Boundary.Center + this.Boundary.Min;
            max = this.Boundary.Center + this.Boundary.Max;
            mesh = MeshGenerator.CuboidFromExtents(min,max);
            h = mesh.Draw(container,'c');
            set(h,"FaceAlpha",0.2);

            for i = 1:numel(this.Children)
                this.Children(i).Draw(container);
            end
        end
    end
    methods (Access = private)
        function [this] = SubDivide(this)
            % This function generates a set of child nodes within the
            % current node.

            % Calculate new ranges, zero'd around the child position
            qSpan = (this.Boundary.Max - this.Boundary.Min)/4;
            c = this.Boundary.Center;

            unitaryCuboid = [
                1, 1, 1;
                -1, 1, 1;
                 1,-1, 1;
                -1,-1, 1;
                 1, 1,-1;
                -1, 1,-1;
                 1,-1,-1;
                -1,-1,-1];
            centroids = unitaryCuboid.*qSpan';

            for i = 1:8
                % Position the child centers
                childCenter = c + centroids(i,:)';
                % Create the new sub-boundary
                childBoundary = AABB( ...
                    childCenter, ...
                    [-1,1]*qSpan(1), ...
                    [-1,1]*qSpan(2), ...
                    [-1,1]*qSpan(3));
                % Assign the new child
                this.Children(i) = OctreeNode(childBoundary,this.Capacity);
            end
 
            % Indicate we have divided
            this.HasDivided = true;
        end
    end
end