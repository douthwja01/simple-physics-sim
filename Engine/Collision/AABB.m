classdef AABB < handle
    % An Axis-Aligned Bounding Box (AABB) collider primitive
    
    properties (Constant)
        Code = ColliderCode.AABB;
    end

    properties
        xInterval = Interval.empty;
        yInterval = Interval.empty;
        zInterval = Interval.empty;
        Parent = Collider.empty;
    end

    methods
        function [this] = AABB(xlim,ylim,zlim)
            % CONSTRUCTOR - Generate a box- collider with unitary axial
            % limits.

            if nargin < 3
                zlim = [-0.5;0.5];
            end
            if nargin < 2
                ylim = [-0.5;0.5];
            end
            if nargin < 1
                xlim = [-0.5;0.5];
            end
            
            this.xInterval = Interval(xlim);
            this.yInterval = Interval(ylim);
            this.zInterval = Interval(zlim);
        end
        % Get/sets
        function set.Parent(this,p)
            assert(isa(p,"Collider"),"Expecting a valid parent collider.");
            this.Parent = p;
        end
        % Operators
        function [r] = plus(obj1,obj2)
            % How addition is done with AABBs.
            if isa(obj1,"AABB")
                assert(isnumeric(obj2),"Expecting a second numeric input.");
                if IsColumn(obj2,3)
                    x = obj1.xInterval + obj2(1);
                    y = obj1.yInterval + obj2(2);
                    z = obj1.zInterval + obj2(3);
                elseif isnumeric(obj2)
                    x = obj1.xInterval + obj2;
                    y = obj1.yInterval + obj2;
                    z = obj1.zInterval + obj2;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end

            if isa(obj2,"AABB")
                assert(isnumeric(obj1),"Expecting a first numeric input.");
                if IsColumn(obj1,3)
                    x = obj2.xInterval + obj1(1);
                    y = obj2.yInterval + obj1(2);
                    z = obj2.zInterval + obj1(3);
                elseif isnumeric(obj1)
                    x = obj2.xInterval + obj1;
                    y = obj2.yInterval + obj1;
                    z = obj2.zInterval + obj1;
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
                    x = obj1.xInterval - obj2(1);
                    y = obj1.yInterval - obj2(2);
                    z = obj1.zInterval - obj2(3);
                elseif isnumeric(obj2)
                    x = obj1.xInterval - obj2;
                    y = obj1.yInterval - obj2;
                    z = obj1.zInterval - obj2;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end

            if isa(obj2,"AABB")
                assert(isnumeric(obj1),"Expecting a first numeric input.");
                if IsColumn(obj1,3)
                    x = obj2.xInterval - obj1(1);
                    y = obj2.yInterval - obj1(2);
                    z = obj2.zInterval - obj1(3);
                elseif isnumeric(obj1)
                    x = obj2.xInterval - obj1;
                    y = obj2.yInterval - obj1;
                    z = obj2.zInterval - obj1;
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
                    x = obj1.xInterval*obj2(1);
                    y = obj1.yInterval*obj2(2);
                    z = obj1.zInterval*obj2(3);
                elseif isnumeric(obj2)
                    x = obj1.xInterval*obj2;
                    y = obj1.yInterval*obj2;
                    z = obj1.zInterval*obj2;
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
                    x = r.xInterval*obj1(1);
                    y = r.yInterval*obj1(2);
                    z = r.zInterval*obj1(3);
                elseif isnumeric(obj1)
                    x = r.xInterval*obj1;
                    y = r.yInterval*obj1;
                    z = r.zInterval*obj1;
                else
                    error("Second input not compatible.");
                end
                r = AABB(x.Range,y.Range,z.Range);
                return;
            end
        end
    end

    % AABB Tools
    methods (Static)
        function [mesh] = CreateMesh(aabb)
            % This function creates a cuboid mesh representing the aabb.
            assert(isa(aabb,"AABB"),"Expecting a valid AABB primitive.");
            % Extents
            mins = [aabb.xInterval.Min;aabb.yInterval.Min;aabb.zInterval.Min];
            maxs = [aabb.xInterval.Max;aabb.yInterval.Max;aabb.zInterval.Max];
            % Create the mesh
            mesh = MeshGenerator.CuboidFromExtents(mins,maxs);
        end
        function [aabb] = EncloseMesh(mesh)
            % This function creates an axis-aligned-bounding box from a
            % given mesh.
            
            % Simply extract the extents
            [xlimits,ylimits,zlimits] = Mesh.Extents(mesh);
            % Create the primitive
            aabb = AABB(xlimits,ylimits,zlimits);
        end
    end
end