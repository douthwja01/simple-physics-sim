classdef CollisionPlane < CollisionPrimitive
    properties
        Normal = [1;0;0];
        Distance = 0;
    end
    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if this plane is colliding with a given point
            % primitive.
            [flag] = point.CheckPlane(this);
        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this plane is colliding with a given sphere
            % primitive.
            flag = sphere.CheckPlane(this);
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if this plane is colliding with a given aabb
            % primitive.
            flag = aabb.CheckPlane(this);
        end
        function [flag] = CheckPlane(this,plane)
            % Check if this plane is colliding with a given plane
            % primitive.

            normalArg = this.Normal.Cross(plane.Normal);
            normalMagnitude = norm(normalArg);
            % If the argument is less than zero
            flag = normalMagnitude > 0 || this.Distance == plane.Distance;
        end
        function [flag] = CheckOBB(this,obb)
            % Check if this plane is colliding with a given obb
            % primitive.
            flag = obb.CheckPlane(this);
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this plane is colliding with a given capsule
            % primitive.
            flag = capsule.CheckPlane(this);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this plane is colliding with a given triangle
            % primitive.

            side_a = this.PlaneEquation(triangle.A);
            side_b = this.PlaneEquation(triangle.B);
            side_c = this.PlaneEquation(triangle.C);
            
            if side_a == 0 && side_b == 0 && side_c == 0 
                flag = true;
                return;
            end
            
            if sign(side_a) == sign(side_b) && sign(side_a) == sign(side_c)
                flag = false;
                return;
            end
            flag = true;
        end
        function [flag] = CheckRay(this,ray)
            % Check if this plane is colliding with a given ray
            % primitive.

            DdotN = dot(ray.Direction,this.Normal);
            % The ray is not in the direction of the plane
            if DdotN >= 0
                flag = false;
                return
            end

            OdotN = dot(ray.Origin,this.Normal);
            % The origin is behind the plane
            t = (this.Distance - OdotN) / DdotN;
            if t < 0
                flag = false;
                return
            end
            % Project contact point
%             contact_point = ray.Origin + ray.Direction *t;
%             hit_info.Update(t, self, contact_point, self.normal);
            
            flag = true;
        end
        function [flag] = CheckLine(this,line)
            % Check if this plane is colliding with a given line
            % primitive.

            dir = line.Finish - line.Start;
            NdotS = dot(this.Normal,line.Start);
            NdotD = dot(this.Normal,dir);
            
            if (NdotD == 0) 
                flag = false;
                return
            end
            t = (this.Distance - NdotS) / NdotD;
            flag = t >= 0 && t <= 1;
        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this plane is colliding with a given mesh
            % primitive.
            flag = mesh.CheckPlane(this);
        end
    end
    methods
        function [closest] = ClosestPoint(point) 
            % This function calculates the closest point on the plane
            % surface to a give point.

            ndot = dot(this.Normal,point);
            dist = ndot - this.Distance;
            scaled_dist = this.Normal * dist;
            closest = point - scaled_dist;
        end
        function [sideBool] = PlaneEquation(vec3)
            % Much like the dot product, this function will return:
            % - +1ish if the value is in front of the plane
            % - 0 if the value is on the plane
            % - -1ish is the value is behind the plane
            argument = dot(vec3,this.Normal);
            sideBool = argument - this.Distance;
        end
        function [plane] = Normalize(this)
            m = norm(this.Normal);
            plane = CollisionPlane(this.Normal/m,this.Distance/m);
        end
    end
end