classdef CollisionOBB < CollisionPrimitive
    properties
        Size = ones(3,1);
        Orientation = eye(3);
        ImaginaryRadius = 1;
    end
    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if this obb is colliding with a given point
            % primitive.

            dir = point.position - this.position;
        
            size_array = this.size.AsLinearArray();
            orientation_array = this.orientation.AsVectorArray();
        
            for i = 1:3
                axis = orientation_array(i);
                dist = dot(dir,axis);
                if abs(dist) > abs(size_array[i]) 
                    flag = false;
                    return
                end
            end
            flag = true;
        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this obb is colliding with a given sphere
            % primitive.

            distance = CollisionTools.Distance(this.Position,sphere.Position);
            
            % Check radial separation
            if distance > this.ImaginaryRadius + sphere.radius 
                flag = false;
                return
            end
            % Get the nearest point to a given point
            nearest = this.NearestPoint(sphere.position);
            dist = CollisionTools.Distance(nearest, sphere.Position);
            flag = dist < sphere.radius;
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if this obb is colliding with a given aabb
            % primitive.

            axes = zeros(6,3);
            axes(1,:) = [1, 0, 0];
            axes(2,:) = [0, 1, 0];
            axes(3,:) = [0, 0, 1];
            axes(4,:) = this.orientation.x;
            axes(5,:) = this.orientation.y;
            axes(6,:) = this.orientation.z;
        
            for i = 1:3 
                for j = 4:6 
                    crossAxes = cross(axes(i,:),axes(j,:));
                    axes = vertcat(axes,crossAxes);
                end
            end
            % Check for no overlap on each axis
            for i = 1:15
                if ~this.CheckAxisOverlap(this, aabb, axes(i,:)) 
                    flag = false;
                    return
                end
            end
        
            flag = true;
        end
        function [flag] = CheckPlane(this,plane)
            % Check if this obb is colliding with a given plane
            % primitive.

            plen = this.Size.x * abs(dot(plane.Normal,this.orientation.x)) + ...
            this.Size.y * abs(dot(plane.Normal,this.orientation.y)) + ...
            this.Size.z * abs(dot(plane.Normal,this.orientation.z));
        
            dist = dot(plane.normal,this.Position) - plane.Distance;
        
            flag = abs(dist) < plen;
        end
        function [flag] = CheckOBB(this,obb)
            % Check if this obb is colliding with a given obb
            % primitive.
            
            my_radius = this.Size.Magnitude();
            other_radius = obb.Size.Magnitude();
       
            distance = CollisionPrimitive.DistanceTo(this.position,obb.position);
            % Simple spherical separation check
            if distance > my_radius + other_radius 
                flag = false;
                return
            end
            
            % Check each axis
            axes = zeros(6,3);
            axes(1,:) = obb.orientation.x;
            axes(2,:) = obb.orientation.y;
            axes(3,:) = obb.orientation.z;
            axes(4,:) = this.orientation.x;
            axes(5,:) = this.orientation.y;
            axes(6,:) = this.orientation.z;
            for i = 1:3 
                for j = 4:6 
                    crossAxes = cross(axes(i,:),axes(j,:));
                    axes = vertcat(axes,crossAxes);
                end
            end
            % Check for no overlap on each axis
            for i = 1:15
                if ~this.CheckAxisOverlap(this, obb, axes(i,:)) 
                    flag = false;
                    return
                end
            end
        
            flag = true;
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this obb is colliding with a given capsule
            % primitive.
            flag = capsule.CheckOBB(this);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this obb is colliding with a given triangle
            % primitive.

            edges = zeros(3,3);
            edges(1,:) = triangle.B - triangle.A;
            edges(2,:) = triangle.C - triangle.B;
            edges(3,:) = triangle.A - triangle.C;
            
            axes = zeros(4,3);
            axes(1,:) = this.orientation.x;
            axes(2,:) = this.orientation.y;
            axes(3,:) = this.orientation.z; 
            axes(4,:) = triangle.GetNormal();
            
            
            for i = 1:3
                for j = 1:3 
                    axes = vertcat(axes, cross(axes(i,:),edges(j,:)));
                end
            end
            % Evaluate each axis against the triangle
            for i = 1:13
                if ~ CollisionPrimitive.CheckAxisOverlap(this, triangle, axes(i,:)) 
                    flag = false;
                    return 
                end
            end
            flag = true;
        end
        function [flag] = CheckRay(this,collisionRay)
            % Check if this obb is colliding with a given ray
            % primitive.

            size_array = this.Size.AsLinearArray();
            direction = this.Position - ray.Origin;
            
            direction_dots = zeros(3,3);
            direction_dots(1,:) = dot(this.orientation.x,collisionRay.direction);
            direction_dots(2,:) = dot(this.orientation.y,collisionRay.direction);
            direction_dots(3,:) = dot(this.orientation.z,collisionRay.direction);
            position_dots = zeros(3,3);
            position_dots(1,:) = dot(this.orientation.x,direction);
            position_dots(2,:) = dot(this.orientation.y,direction);
            position_dots(3,:) = dot(this.orientation.z,direction);
            
            t = array_create(6, 0);
            
            for i = 1:3
                if direction_dots[i] == 0 
                    if ((-position_dots[i] - size_array[i]) > 0 || (-position_dots[i] + size_array[i]) < 0) {
                        return false;
                    end
                    direction_dots[i] = 0.0001;
                end
                
                t[i * 2 + 0] = (position_dots[i] + size_array[i]) / direction_dots[i];
                t[i * 2 + 1] = (position_dots[i] - size_array[i]) / direction_dots[i];
           end
            
            var tmin = max(
                min(t[0], t[1]),
                min(t[2], t[3]),
                min(t[4], t[5])
            );
            
            var tmax = min(
                max(t[0], t[1]),
                max(t[2], t[3]),
                max(t[4], t[5])
            );
            
            if tmax < 0 
                flag = false;
                return
            end

            if tmin > tmax
                flag = false;
                return
            end
            
            if (hit_info) 
                var contact_distance = (tmin < 0) ? tmax : tmin;
                var contact_normal = new Vector3(0, 0, 0);
                
                var contact_point = ray.origin.Add(ray.direction.Mul(contact_distance));
                
                var possible_normals = [
                    self.orientation.x,
                    self.orientation.x.Mul(-1),
                    self.orientation.y,
                    self.orientation.y.Mul(-1),
                    self.orientation.z,
                    self.orientation.z.Mul(-1),
                ];
                
                for i = 1:6
                    if contact_distance == t[i]
                        contact_normal = possible_normals[i];
                    end
                end
                
                hit_info.Update(contact_distance, self, contact_point, contact_normal);
            end
            
            flag = true;
        end
        function [flag] = CheckLine(this,line)
            % Check if this obb is colliding with a given line
            % primitive.

            direction = line.finish -line.start;
            direction = direction/norm(direction);

            collisionRay = CollisionRay(line.start, direction);
            hit_info = RaycastHitInformation();

            if this.CheckRay(collisionRay, hit_info)
                flag = hit_info.Distance <= line.Length();
                return
            end
            flag = false;
        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this obb is colliding with a given mesh
            % primitive.
            flag = mesh.CheckOBB(this);
        end
    end
    % Support methods
    methods
        function [axisInterval] = GetAxisInterval(this,axis)
            % This function creates an interval defining the projection of
            % this shape onto a given axis.

            % Ensure same size
            axis = axis'; 
            % Define vertex data from limits
            vertices = this.GetVertices();
            % We need to loop through these points and get the minimum and
            % maximums on the provided axis (axis is assumed to be
            % arbitrary)
            imin = inf;
            imax = -inf;

            for i = 1:size(vertices,1)
                projection = dot(axis,vertices);
                % If the projection is greater record it.
                if projection > imax
                    imax= projection;
                end
                % If the projection is smaller record it.
                if projection < imin
                    imin = projection;
                end
            end
            % Return an interval capturing it.
            axisInterval = Interval(imin,imax);
        end
        function [vertices] = GetVertices(this)
            % This function returns the bounds of the AABB as vertices.

%             % Express the bounds as dimension ranges
%             xRange = this.Bounds(1).Range;
%             yRange = this.Bounds(2).Range;
%             zRange = this.Bounds(2).Range;
%             % Define vertex data from limits
%             vertices = zeros(8,3);
%             vertices(1,:) = [xRange(2),yRange(2),zRange(2)];
%             vertices(2,:) = [xRange(2),yRange(2),zRange(1)];
%             vertices(3,:) = [xRange(2),yRange(1),zRange(1)];
%             vertices(4,:) = [xRange(2),yRange(1),zRange(2)];
%             vertices(5,:) = [xRange(1),yRange(1),zRange(1)];
%             vertices(6,:) = [xRange(1),yRange(1),zRange(2)];
%             vertices(7,:) = [xRange(1),yRange(2),zRange(2)];
%             vertices(8,:) = [xRange(1),yRange(2),zRange(1)];
        end
    end
end