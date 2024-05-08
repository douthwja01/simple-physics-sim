classdef AABB < handle
    % An Axis-Aligned Bounding Box (AABB) collider primitive
    
    properties (Dependent) 
        Min;
        Max;
    end 
    % Intervals in each dimensions
    properties (SetAccess = private)
        Bounds = Interval.empty(0,3);
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
            this.Bounds(1) = Interval(xRange(1),xRange(2));
            this.Bounds(2) = Interval(yRange(1),yRange(2));
            this.Bounds(3) = Interval(zRange(1),zRange(2));
        end
        % Get/sets
        function [m] = get.Min(this)
            m = [this.Bounds.Min];
        end
        function set.Min(this,m)
            assert(numel(m)==3,"Expecting a valid 3D minimum array [3x1].");
            this.Bounds(1).Min = m(1);
            this.Bounds(2).Min = m(2);
            this.Bounds(3).Min = m(3);
        end
        function [m] = get.Max(this)
            m = [this.Bounds.Max];
        end
        function set.Max(this,m)
            assert(numel(m)==3,"Expecting a valid 3D maximum array [3x1].");
            this.Bounds(1).Max = m(1);
            this.Bounds(2).Max = m(2);
            this.Bounds(3).Max = m(3);
        end
        % Operators
        function [result] = plus(obj1,obj2)
            % How addition is done with AABBs.
        
            result = AABB();
            if isa(obj1,"AABB")
                switch true
                    case isscalar(obj2)
                        result.Bounds(1) = obj1.Bounds(1) + obj2;
                        result.Bounds(2) = obj1.Bounds(2) + obj2;
                        result.Bounds(3) = obj1.Bounds(3) + obj2;
                    case IsColumn(obj2,3)
                        result.Bounds(1) = obj1.Bounds(1) + obj2(1);
                        result.Bounds(2) = obj1.Bounds(2) + obj2(2);
                        result.Bounds(3) = obj1.Bounds(3) + obj2(3);
                    otherwise
                        error("Input not correct.");
                end
            end

            if isa(obj2,"AABB")
                switch true
                    case isscalar(obj1)
                        result.Bounds(1) = obj2.Bounds(1) + obj1;
                        result.Bounds(2) = obj2.Bounds(2) + obj1;
                        result.Bounds(3) = obj2.Bounds(3) + obj1;
                    case IsColumn(obj1,3)
                        result.Bounds(1) = obj2.Bounds(1) + obj1(1);
                        result.Bounds(2) = obj2.Bounds(2) + obj1(2);
                        result.Bounds(3) = obj2.Bounds(3) + obj1(3);
                    otherwise
                        error("Input not correct.");
                end
            end
        end
        function [result] = minus(obj1,obj2)
            % How subtraction is done with AABBs.
            
            result = AABB();

            if isa(obj1,"AABB")
                switch true
                    case isscalar(obj2)
                        result.Bounds(1) = obj1.Bounds(1) - obj2;
                        result.Bounds(2) = obj1.Bounds(2) - obj2;
                        result.Bounds(3) = obj1.Bounds(3) - obj2;
                    case IsColumn(obj2,3)
                        result.Bounds(1) = obj1.Bounds(1) - obj2(1);
                        result.Bounds(2) = obj1.Bounds(2) - obj2(2);
                        result.Bounds(3) = obj1.Bounds(3) - obj2(3);
                    otherwise
                        error("Input not correct.");
                end
            end

            if isa(obj2,"AABB")
                switch true
                    case isscalar(obj1)
                        result.Bounds(1) = obj2.Bounds(1) - obj1;
                        result.Bounds(2) = obj2.Bounds(2) - obj1;
                        result.Bounds(3) = obj2.Bounds(3) - obj1;
                    case IsColumn(obj1,3)
                        result.Bounds(1) = obj2.Bounds(1) - obj1(1);
                        result.Bounds(2) = obj2.Bounds(2) - obj1(2);
                        result.Bounds(3) = obj2.Bounds(3) - obj1(3);
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
                % Create a temporary output copy (includes Cid)
                switch true
                    case isscalar(obj2)
                        result.Bounds(1) = obj1.Bounds(1)*obj2;
                        result.Bounds(2) = obj1.Bounds(2)*obj2;
                        result.Bounds(3) = obj1.Bounds(3)*obj2;
                    case IsColumn(obj2,3)
                        result.Bounds(1) = obj1.Bounds(1)*obj2(1);
                        result.Bounds(2) = obj1.Bounds(2)*obj2(2);
                        result.Bounds(3) = obj1.Bounds(3)*obj2(3);
                    otherwise
                        error("Input not correct.");
                end

                return;
            end

            if isa(obj2,"AABB")
                % Sanity check
                assert(isnumeric(obj1),"Expecting a second numeric input.");
                switch true
                    case isscalar(obj1)
                        result.Bounds(1) = obj2.Bounds(1)*obj1;
                        result.Bounds(2) = obj2.Bounds(2)*obj1;
                        result.Bounds(3) = obj2.Bounds(3)*obj1;
                    case IsColumn(obj1,3)
                        result.Bounds(1) = obj2.Bounds(1)*obj1(1);
                        result.Bounds(2) = obj2.Bounds(2)*obj1(2);
                        result.Bounds(3) = obj2.Bounds(3)*obj1(3);
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
            if ~this.Bounds(1).IntersectRay(ray.Origin(1), ray.Direction(1), t_enter, t_exit) 
                return;    
            end
            % Y-axis
            if ~this.Bounds(2).IntersectRay(ray.Origin(2), ray.Direction(2), t_enter, t_exit) 
                return;    
            end
            % Z-axis    
            if ~this.Bounds(3).IntersectRay(ray.Origin(3), ray.Direction(3), t_enter, t_exit) 
                return;  
            end

            % All axes intersect
            flag = true;
        end
        function [flag] = IntersectAABB(this,other)
            % Test for overlap of two AABBs

            flag = false;
            if ~this.Bounds(1).IntersectInterval(other.Bounds(1))
                return
            end
            if ~this.Bounds(2).IntersectInterval(other.Bounds(2))
                return
            end
            if ~this.Bounds(3).IntersectInterval(other.Bounds(3))
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
                    range = this.Bounds(1).Range;
                case AxisCode.Y
                    range = this.Bounds(2).Range;
                case AxisCode.Z
                    range = this.Bounds(3).Range;
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
                    flag = this.Bounds(1).Contains(value);
                case AxisCode.Y
                    flag = this.Bounds(2).Contains(value);
                case AxisCode.Z
                    flag = this.Bounds(3).Contains(value);
                otherwise
                    error("Invalid axis-code for AABB checks.");
            end
        end        
        % General
        function [mesh] = ToMesh(this)
            % Create a mesh from the current AABB instance.
            mesh = MeshExtensions.CuboidFromExtents(this.Min,this.Max);
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