
classdef CollisionCapsule < CollisionPrimitive
    properties
        Line = CollisionLine();
        Radius = 1;
    end
    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if this capsule is colliding with a given point
            % primitive.
            
            % Get the nearest point on the line
            nearestLinePoint = this.Line.NearestPoint(point.Position);
            distance = CollisionPrimitive.DistanceTo(nearestLinePoint,point.Position);
            % If this exceeds the radius
            flag = distance < this.Radius;
        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this capsule is colliding with a given sphere
            % primitive.
            
            % Get the point on the line
            nearestLinePoint = this.Line.NearestPoint(sphere.Position);
            distance = CollisionPrimitive.DistanceTo(nearestLinePoint,sphere.Position);
            % Get the sphere-separation
            flag = distance < (this.Radius + sphere.Radius);
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if this capsule is colliding with a given aabb
            % primitive.
            
            % Check the sphere at the start of the capsule
            startSphere = CollisionSphere(this.Line.Start, this.Radius);
            % Sphere-AABB check
            if startSphere.CheckAABB(aabb)
                flag = true;
                return
            end
            % Checke the sphere at the end of the capsule
            finishSphere = CollisionSphere(this.Line.Finish, this.Radius);
            if finishSphere.CheckAABB(aabb)
                flag = true;
                return
            end

            % Get the AABB edges
            edges = aabb.GetEdges();
            
            for i = 1:size(edges,1)
                edge = edges(i,:);

                nearestLineToEdge = edge.NearestConnectionToLine(this.Line);
                nearestPointToLine = this.Line.NearestPoint(nearestLineToEdge.start);
                start_distance = CollisionPrimitive.DistanceTo(nearestPointToLine,nearestLineToEdge.start);
                
                % Check 
                if start_distance == 0
                    test_sphere = CollisionSphere(nearestLineToEdge.Start,this.Radius);
                    if test_sphere.CheckAABB(aabb)
                        flag = true;
                        return
                    end
                else
                    test_sphere = CollisionSphere(nearestLineToEdge.Finish,this.Radius);
                    if test_sphere.CheckAABB(aabb)
                        flag = true;
                        return
                    end
                end
            end
            flag = false;
        end
        function [flag] = CheckPlane(this,plane)
            % Check if this capsule is colliding with a given plane
            % primitive.

            nearest_start = plane.NearestPoint(this.Line.start);

            % Check if the plane is within the radius of the start
            if this.Line.Start.DistanceTo(nearest_start) < this.Radius
                flag = true;
                return
            end
        
            nearest_finish = plane.NearestPoint(this.Line.Finish);
            if this.Line.Finish.DistanceTo(nearest_finish) < this.radius
                flag = true;
                return
            end
            % Check the line intersection
            flag = this.Line.CheckPlane(plane);
        end
        function [flag] = CheckOBB(this,obb)
            % Check if this capsule is colliding with a given obb
            % primitive.

            startSphere = CollisionSphere(this.Line.Start, this.Radius);
            if startSphere.CheckOBB(obb)
                flag = true;
                return
            end
            
            finishSphere = CollisionSphere(this.Line.Finish, this.Radius);
            if finishSphere.CheckOBB(obb)
                flag = true;
                return 
            end
            
            edges = obb.GetEdges();
            
            for i = 1:size(edges,1)
                edge = edges(i);

                nearestLineToEdge = edge.NearestConnectionToLine(this.Line);
                nearestPointToLine = this.Line.NearestPoint(nearestLineToEdge.start);
                start_distance = nearestPointToLine.DistanceTo(nearestLineToEdge.start);
                
                if start_distance == 0 
                    testSphere = CollisionSphere(nearestLineToEdge.start,this.Radius);
                    if testSphere.CheckOBB(obb)
                        flag = true;
                        return
                    end
               else 
                    testSphere = CollisionSphere(nearestLineToEdge.Finish,this.Radius);
                    if testSphere.CheckOBB(obb)
                        flag = true;
                        return
                    end
                end
            end
            
            flag = false;
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this capsule is colliding with a given capsule
            % primitive.

            % Get the 
            connectingLine = this.Line.NearestConnectionToLine(capsule.Line);
            distance = norm(connectingLine);%.Length();
            % Is the distance greater radius
            flag = distance < (this.Radius + capsule.Radius);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this capsule is colliding with a given triangle
            % primitive.

            nearestPointStart = triangle.NearestPoint(this.Line.Start);
            capsuleLineNearestPointStart = this.Line.NearestPoint(nearestPointStart);
            if capsuleLineNearestPointStart.DistanceTo(nearestPointStart) < this.Radius
                flag = true;
                return
            end
            nearestPointFinish = triangle.NearestPoint(this.Line.finish);
            capsuleLineNearestPointFinish = this.Line.NearestPoint(nearestPointFinish);
            if capsuleLineNearestPointFinish.DistanceTo(nearestPointFinish) < this.Radius) 
                flag = true;
                return
            end
            flag = false;
        end
        function [flag] = CheckRay(this,ray)
            % Check if this capsule is colliding with a given ray
            % primitive.

            capsuleDir = this.Line.Finish - this.Line.Start;
            relativeRayOrigin = ray.Origin - this.Line.Start;
            
            baba = dot(capsuleDir,capsuleDir);
            bard = dot(capsuleDir,ray.Direction);
            baoa = dot(capsuleDir,relativeRayOrigin);
            rdoa = dot(ray.Direction,relativeRayOrigin);
            oaoa = dot(relativeRayOrigin,relativeRayOrigin);
            
            a = baba - sqr(bard);
            b = baba * rdoa - baoa * bard;
            c = baba * oaoa - sqr(baoa) - sqr(this.Radius) * baba;
            h = sqr(b) - a * c;
            
            if h > 0
                t = (-b - sqrt(h)) / a;
                why = baoa + t * bard;
                
                if why > 0 && why < baba
%                     contact_point = ray.origin.Add(ray.direction.Mul(t));
%                     nearest_inner_point = self.line.NearestPoint(contact_point);
%                     temp = contact_point - nearest_inner_point;
%                     contact_normal = temp/norm(temp);
                    %hit_info.Update(t, self, contact_point, contact_normal);
                    flag = true;
                    return;
                end
                
                %oc = (why <= 0) ? relative_ray_origin : ray.origin.Sub(self.line.finish);
                if why <= 0
                    oc = relativeRayOrigin;
                else
                    oc = ray.Origin - this.Line.Finish;
                end

                b = dot(ray.Direction,oc);
                c = dot(oc,oc) - sqr(this.Radius);
                h = sqr(b) - c;
                
                if h > 0
%                     t = -b - sqrt(h);
%                     contact_point = ray.origin.Add(ray.direction.Mul(t));
%                     nearest_inner_point = self.line.NearestPoint(contact_point);
%                     contact_normal = contact_point.Sub(nearest_inner_point).Normalize();
%                     hit_info.Update(t, self, contact_point, contact_normal);
                    flag = true;
                    return
                end
             end
            
            flag = false;
        end
        function [flag] = CheckLine(this,line)
            % Check if this capsule is colliding with a given line
            % primitive.
            closestLine = this.Line.NearestConnectionToLine(line);
            flag = closestLine.Length() < this.Radius;
        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this capsule is colliding with a given mesh
            % primitive.
            flag = mesh.CheckCapsule(this);
        end
    end
end