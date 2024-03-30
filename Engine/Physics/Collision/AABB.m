classdef AABB < Boundary
    % An Axis-Aligned Bounding Box (AABB) collider primitive
    
    properties (Constant)
        Code = ColliderCode.AABB;
    end
    properties (Dependent) 
        Min;
        Max;
    end  
    properties
        Cid = uint32.empty;      % Identity code
    end
    % Intervals in each dimensions
    properties (SetAccess = private)
        xBoundary = Interval.empty;
        yBoundary = Interval.empty;
        zBoundary = Interval.empty;
    end
    methods
        % Constructor
        function [this] = AABB(xRange,yRange,zRange)
            % CONSTRUCTOR - Generate a box- collider with unitary axial
            % limits.
            % center - 3D center position.
            % xRange - The x-variation around the center.
            % yRange - The y-variation around the center.
            % zRange - The z-variation around the center.

            % Input check
            if nargin < 1
                xRange = [-1;1];
            end
            if nargin < 2
                yRange = [-1;1];
            end
            if nargin < 3
                zRange = [-1;1];
            end

            % Parameterise
            this.xBoundary = Interval(xRange(1),xRange(2));
            this.yBoundary = Interval(yRange(1),yRange(2));
            this.zBoundary = Interval(zRange(1),zRange(2));
        end
        % Get/sets
        function set.Cid(this,cid)
            assert(isa(cid,"uint32"),"Expecting a 'uint32' cid code.");
            this.Cid = cid;
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
        % Operators
        function [result] = plus(obj1,obj2)
            % How addition is done with AABBs.
        
            result = AABB();

            if isa(obj1,"AABB")
                % Pass Cid
                result.Cid = obj1.Cid;
                switch true
                    case isscalar(obj2)
                        result.xBoundary = obj1.xBoundary + obj2;
                        result.yBoundary = obj1.yBoundary + obj2;
                        result.zBoundary = obj1.zBoundary + obj2;
                    case IsColumn(obj2,3)
                        result.xBoundary = obj1.xBoundary + obj2(1);
                        result.yBoundary = obj1.yBoundary + obj2(2);
                        result.zBoundary = obj1.zBoundary + obj2(3);
                    otherwise
                        error("Input not correct.");
                end
            end

            if isa(obj2,"AABB")
                % Pass Cid
                result.Cid = obj2.Cid;
                switch true
                    case isscalar(obj1)
                        result.xBoundary = obj2.xBoundary + obj1;
                        result.yBoundary = obj2.yBoundary + obj1;
                        result.zBoundary = obj2.zBoundary + obj1;
                    case IsColumn(obj1,3)
                        result.xBoundary = obj2.xBoundary + obj1(1);
                        result.yBoundary = obj2.yBoundary + obj1(2);
                        result.zBoundary = obj2.zBoundary + obj1(3);
                    otherwise
                        error("Input not correct.");
                end
            end
        end
        function [result] = minus(obj1,obj2)
            % How subtraction is done with AABBs.
            
            result = AABB();

            if isa(obj1,"AABB")
                % Pass Cid
                result.Cid = obj1.Cid;
                switch true
                    case isscalar(obj2)
                        result.xBoundary = obj1.xBoundary - obj2;
                        result.yBoundary = obj1.yBoundary - obj2;
                        result.zBoundary = obj1.zBoundary - obj2;
                    case IsColumn(obj2,3)
                        result.xBoundary = obj1.xBoundary - obj2(1);
                        result.yBoundary = obj1.yBoundary - obj2(2);
                        result.zBoundary = obj1.zBoundary - obj2(3);
                    otherwise
                        error("Input not correct.");
                end
            end

            if isa(obj2,"AABB")
                % Pass Cid
                result.Cid = obj2.Cid;
                switch true
                    case isscalar(obj1)
                        result.xBoundary = obj2.xBoundary - obj1;
                        result.yBoundary = obj2.yBoundary - obj1;
                        result.zBoundary = obj2.zBoundary - obj1;
                    case IsColumn(obj1,3)
                        result.xBoundary = obj2.xBoundary - obj1(1);
                        result.yBoundary = obj2.yBoundary - obj1(2);
                        result.zBoundary = obj2.zBoundary - obj1(3);
                    otherwise
                        error("Input not correct.");
                end
            end
        end
        function [result] = mtimes(obj1,obj2)
            % How multiplication is done with AABBs.
            
            result = AABB();
            
            if isa(obj1,"AABB")
                % Sanity check
                assert(isnumeric(obj2),"Expecting a second numeric input.");
                % Pass Cid
                result.Cid = obj1.Cid;
                % Create a temporary output copy (includes Cid)
                switch true
                    case isscalar(obj2)
                        result.xBoundary = obj1.xBoundary*obj2;
                        result.yBoundary = obj1.yBoundary*obj2;
                        result.zBoundary = obj1.zBoundary*obj2;
                    case IsColumn(obj2,3)
                        result.xBoundary = obj1.xBoundary*obj2(1);
                        result.yBoundary = obj1.yBoundary*obj2(2);
                        result.zBoundary = obj1.zBoundary*obj2(3);
                    otherwise
                        error("Input not correct.");
                end

                return;
            end

            if isa(obj2,"AABB")
                % Sanity check
                assert(isnumeric(obj1),"Expecting a second numeric input.");
                % Pass Cid
                result.Cid = obj2.Cid;
                switch true
                    case isscalar(obj1)
                        result.xBoundary = obj2.xBoundary*obj1;
                        result.yBoundary = obj2.yBoundary*obj1;
                        result.zBoundary = obj2.zBoundary*obj1;
                    case IsColumn(obj1,3)
                        result.xBoundary = obj2.xBoundary*obj1(1);
                        result.yBoundary = obj2.yBoundary*obj1(2);
                        result.zBoundary = obj2.zBoundary*obj1(3);
                    otherwise
                        error("Input not correct.");
                end
                return;
            end
        end
    end
    methods
        % Boundary checks       
        function [flag] = IntersectRay(this,ray,t_enter,t_exit)
            % Intersect the AABB with a ray object.

            % Sanity check
            if nargin < 4
                t_exit = ray.Magnitude;
            end
            if nargin < 3
                t_enter = 0;
            end

            flag = false;
            % X-axis check
            if ~this.xBoundary.IntersectRay(ray.Origin(1), ray.Direction(1), t_enter, t_exit) 
                return;    
            end
            % Y-axis
            if ~this.yBoundary.IntersectRay(ray.Origin(2), ray.Direction(2), t_enter, t_exit) 
                return;    
            end
            % Z-axis    
            if ~this.zBoundary.IntersectRay(ray.Origin(3), ray.Direction(3), t_enter, t_exit) 
                return;  
            end

            % All axes intersect
            flag = true;
        end
        function [flag] = IntersectAABB(this,other)
            % Test for overlap of two AABBs

            flag = false;
            if ~this.xBoundary.IntersectInterval(other.xBoundary)
                return
            end
            if ~this.yBoundary.IntersectInterval(other.yBoundary)
                return
            end
            if ~this.zBoundary.IntersectInterval(other.zBoundary)
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
        % General
        function [mesh] = ToMesh(this)
            % Create a mesh from the current AABB instance.
            mesh = MeshExtensions.CuboidFromExtents(this.Min,this.Max);
        end
        function [h] = Draw(this,container,colour)
            % Draw this mesh to a given graphical container
            if nargin < 3
                colour = 'b';
            end
            if nargin < 2
                container = gca;
            end
            mesh = this.ToMesh();
            h = mesh.Draw(container,colour);
        end
    end
    % AABB Tools
    methods (Static)
        function [aabb] = FromMesh(mesh)
            % This function creates an axis-aligned-bounding box from a
            % given mesh.
            
            % Sanity check
            assert(isa(mesh,"Mesh"),"Expecting a valid mesh class.");

            % Simply extract the extents
            [xlimits,ylimits,zlimits] = Mesh.Extents(mesh);
            % Create the primitive
            aabb = AABB(xlimits,ylimits,zlimits);
        end
    end
end