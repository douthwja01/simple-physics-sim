
classdef OctreeNode < handle

    properties
        Position = zeros(3,1);
        Size = 1;
        Children = OctreeNode.empty;
        Points = {};
    end

    methods
        function [this] = OctreeNode(center,size)
            % The constructor for an octree-node        

            assert(IsColumn(center,"Expecting a valid 3D center position."));
            assert(isscalar(size),"Expecting a scalar numeric size.");
            this.Position = center;
            this.Size = size;
        end
        function [flag] = InsertCollider(this,collider)
            % Allow the addition of a collider to the octree.

            if numel(this.Children) == 0
                
            end
        end
        function [flag] = InsertPoint(this,point)

            flag = false;
            % Bounds
            mins = this.Position - this.Size;
            maxs = this.Position + this.Size;

            if (mins(1) > point(1) || maxs(1) < point(1))
                return
            end
            if (mins(2) > point(2) || maxs(2) < point(2))
                return
            end
            if (mins(3) > point(3) || maxs(3) < point(3))
                return
            end
            flag = true;

            % Append the point
            this.Points = vertcat(this.Points,point);
        end
        function [this] = SubDivide(this)
            % This function generates a set of child nodes within the
            % current node.

            childSize = this.Size/2;
            childNodeCenters = this.Position + ...
                [1, 1, 1;
                -1, 1, 1;
                 1,-1, 1;
                -1,-1, 1;
                 1, 1,-1;
                -1, 1,-1;
                 1,-1,-1;
                -1,-1,-1];

            % Create a the child-nodes
            this.Children(1) = OctreeNode(childNodeCenters(:,1),childSize);
            this.Children(1) = OctreeNode(childNodeCenters(:,2),childSize); 
            this.Children(1) = OctreeNode(childNodeCenters(:,3),childSize); 
            this.Children(1) = OctreeNode(childNodeCenters(:,4),childSize); 
            this.Children(1) = OctreeNode(childNodeCenters(:,5),childSize); 
            this.Children(1) = OctreeNode(childNodeCenters(:,6),childSize); 
            this.Children(1) = OctreeNode(childNodeCenters(:,7),childSize); 
            this.Children(1) = OctreeNode(childNodeCenters(:,8),childSize); 
        end

        function [h] = Draw(this,container)
            % Allow drawing of the octree node.

            if nargin < 2
                container = gca;
            end
            
            % Draw the cuboid
            mesh = MeshGenerator.CuboidFromExtents(-ones(3,1)*this.Size,ones(3,1)*this.Size);
            mesh.Vertices = mesh.Vertices + this.Position';
            h = mesh.Draw(container,'g');
            set(h,"FaceAlpha",0.2);

            for i = 1:numel(this.Children)
                this.Children(i).Draw();
            end
        end
    end
end