classdef AABB < Boundary
    % An Axis-Aligned Bounding Box (AABB) collider primitive
    
    properties (Constant)
        Code = ColliderCode.AABB;
    end
    properties (Dependent) 
        Center;
        Min;
        Max;
        Range;
    end  
    properties
        Parent = Collider.empty;
    end
    % Intervals relative to center
    properties (Access = private)
        xBoundary = Interval.empty;
        yBoundary = Interval.empty;
        zBoundary = Interval.empty;
    end
    methods
        function [this] = AABB(center,xRange,yRange,zRange)
            % CONSTRUCTOR - Generate a box- collider with unitary axial
            % limits.
            % center - 3D center position.
            % xRange - The x-variation around the center.
            % yRange - The y-variation around the center.
            % zRange - The z-variation around the center.

            % Input check
            if nargin < 1
                center = zeros(3,1);
            end
            if nargin < 2
                xRange = [-1;1];
            end
            if nargin < 3
                yRange = [-1;1];
            end
            if nargin < 4
                zRange = [-1;1];
            end

            % Parameterise
            this.xBoundary = Interval(center(1),xRange(1),xRange(2));
            this.yBoundary = Interval(center(2),yRange(1),yRange(2));
            this.zBoundary = Interval(center(3),zRange(1),zRange(2));
        end
        % Get/sets
        function set.Parent(this,p)
            assert(isa(p,"Collider"),"Expecting a valid parent collider.");
            this.Parent = p;
        end
        function [p] = get.Center(this)
            p = [this.xBoundary.Center;
                this.yBoundary.Center;
                this.zBoundary.Center];
        end
        function set.Center(this,p)
            assert(numel(p) == 3,"Expecting a valid 3D point.");
            this.xBoundary.Center = p(1);
            this.yBoundary.Center = p(2);
            this.zBoundary.Center = p(3);
        end
        function [m] = get.Min(this)
            m = [this.xBoundary.Min;
                this.yBoundary.Min;
                this.zBoundary.Min];
        end
        function set.Min(this,m)
            assert(numel(m)==3,"Expecting a valid 3D minimum array [3x1].");
            this.xBoundary.Min = m(1);
            this.yBoundary.Min = m(2);
            this.zBoundary.Min = m(3);
        end
        function [m] = get.Max(this)
            m = [this.xBoundary.Max;
                this.yBoundary.Max;
                this.zBoundary.Max];
        end
        function set.Max(this,m)
            assert(numel(m)==3,"Expecting a valid 3D maximum array [3x1].");
            this.xBoundary.Max = m(1);
            this.yBoundary.Max = m(2);
            this.zBoundary.Max = m(3);
        end
        function [r] = get.Range(this)
            r = [this.xBoundary.Range;
                this.yBoundary.Range;
                this.zBoundary.Range];
        end
        % Operators
        function [r] = plus(obj1,obj2)
            % How addition is done with AABBs.
            if isa(obj1,"AABB")
                assert(isnumeric(obj2),"Expecting a second numeric input.");
                if IsColumn(obj2,3)
                    x = obj1.xBoundary + obj2(1);
                    y = obj1.yBoundary + obj2(2);
                    z = obj1.zBoundary + obj2(3);
                elseif isnumeric(obj2)
                    x = obj1.xBoundary + obj2;
                    y = obj1.yBoundary + obj2;
                    z = obj1.zBoundary + obj2;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end

            if isa(obj2,"AABB")
                assert(isnumeric(obj1),"Expecting a first numeric input.");
                if IsColumn(obj1,3)
                    x = obj2.xBoundary + obj1(1);
                    y = obj2.yBoundary + obj1(2);
                    z = obj2.zBoundary + obj1(3);
                elseif isnumeric(obj1)
                    x = obj2.xBoundary + obj1;
                    y = obj2.yBoundary + obj1;
                    z = obj2.zBoundary + obj1;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end
        end
        function [r] = minus(obj1,obj2)
            % How subtraction is done with AABBs.
            
            if isa(obj1,"AABB")
                assert(isnumeric(obj2),"Expecting a second numeric input.");
                if IsColumn(obj2,3)
                    x = obj1.xBoundary - obj2(1);
                    y = obj1.yBoundary - obj2(2);
                    z = obj1.zBoundary - obj2(3);
                elseif isnumeric(obj2)
                    x = obj1.xBoundary - obj2;
                    y = obj1.yBoundary - obj2;
                    z = obj1.zBoundary - obj2;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end

            if isa(obj2,"AABB")
                assert(isnumeric(obj1),"Expecting a first numeric input.");
                if IsColumn(obj1,3)
                    x = obj2.xBoundary - obj1(1);
                    y = obj2.yBoundary - obj1(2);
                    z = obj2.zBoundary - obj1(3);
                elseif isnumeric(obj1)
                    x = obj2.xBoundary - obj1;
                    y = obj2.yBoundary - obj1;
                    z = obj2.zBoundary - obj1;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end
        end
        function [r] = mtimes(obj1,obj2)
            % How multiplication is done with AABBs.
            if isa(obj1,"AABB")
                assert(isnumeric(obj2),"Expecting a second numeric input.");
                
                if IsColumn(obj2,3)
                    x = obj1.xBoundary*obj2(1);
                    y = obj1.yBoundary*obj2(2);
                    z = obj1.zBoundary*obj2(3);
                elseif isnumeric(obj2)
                    x = obj1.xBoundary*obj2;
                    y = obj1.yBoundary*obj2;
                    z = obj1.zBoundary*obj2;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end

            if isa(obj2,"AABB")
                assert(isnumeric(obj1),"Expecting a first numeric input.");
                r = obj2;
                if IsColumn(obj1,3)
                    x = r.xBoundary*obj1(1);
                    y = r.yBoundary*obj1(2);
                    z = r.zBoundary*obj1(3);
                elseif isnumeric(obj1)
                    x = r.xBoundary*obj1;
                    y = r.yBoundary*obj1;
                    z = r.zBoundary*obj1;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end
        end
    end
    methods
        function [range] = GetAxisRange(this,axisCode)
            % Get the axis range
            assert(isa(axisCode,"AxisCode"),"Expecting a valid axis-code enum.");
            switch axisCode
                case AxisCode.X
                    range = this.xBoundary.Range;
                case AxisCode.Y
                    range = this.yBoundary.Range;
                case AxisCode.Z
                    range = this.zBoundary.Range;
                otherwise
                    error("Invalid axis-code for AABB range export.");
            end
        end
        function [flag] = Intersects(this,other)
            % Test for overlap of two AABBs

            flag = false;
            if ~this.xBoundary.Intersects(other.xBoundary)
                return
            end
            if ~this.yBoundary.Intersects(other.yBoundary)
                return
            end
            if ~this.zBoundary.Intersects(other.zBoundary)
                return
            end
            flag = true;
        end
        function [flag] = Contains(this,point)
            % Preform an AABB inclusion check 
            % (sequencial axis satisfaction)
            assert(IsColumn(point,3),"Expecting a valid cartesian point [3x1].");
            
            flag = false;
            if ~this.AxisContains(AxisCode.X,point(1))
                return;
            end
            if ~this.AxisContains(AxisCode.Y,point(2))
                return;
            end
            if ~this.AxisContains(AxisCode.Z,point(3))
                return;
            end
            flag = true;
        end
        function [flag] = AxisContains(this,axisCode,value)
            % Test an individual axis for a value occupancy.
            
            assert(isa(axisCode,"AxisCode"),"Expecting a valid axis-code enum.");
            assert(isscalar(value),"Expecting a scalar (assumed dimensional) value to compare.");

            switch axisCode
                case AxisCode.X
                    flag = this.xBoundary.Contains(value);
                case AxisCode.Y
                    flag = this.yBoundary.Contains(value);
                case AxisCode.Z
                    flag = this.zBoundary.Contains(value);
                otherwise
                    error("Invalid axis-code for AABB checks.");
            end
        end
        function [h] = Draw(this,container,colour)
            % Draw this mesh to a given graphical container
            if nargin < 3
                colour = 'b';
            end
            if nargin < 2
                container = gca;
            end
            mesh = CreateMesh(this);
            h = mesh.Draw(container,colour);
        end
    end
    % AABB Tools
    methods (Static)
        function [mesh] = CreateMesh(aabb)
            % This function creates a cuboid mesh representing the aabb.
            assert(isa(aabb,"AABB"),"Expecting a valid AABB primitive.");
            % Extents
            mins = [aabb.xBoundary.Min;aabb.yBoundary.Min;aabb.zBoundary.Min];
            maxs = [aabb.xBoundary.Max;aabb.yBoundary.Max;aabb.zBoundary.Max];
            % Create the mesh
            mesh = MeshGenerator.CuboidFromExtents(mins,maxs);
            mesh.Origin = aabb.Center;
        end
        function [aabb] = EncloseMesh(mesh)
            % This function creates an axis-aligned-bounding box from a
            % given mesh.
            
            % Simply extract the extents
            [xlimits,ylimits,zlimits] = Mesh.Extents(mesh);
            % Create the primitive
            aabb = AABB(mesh.Origin,xlimits,ylimits,zlimits);
        end
    end
end