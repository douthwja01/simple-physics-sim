
classdef CollisionLine < CollisionPrimitive
    properties
        Start = zeros(3,1);
        Finish = zeros(3,1);
    end
    % Core methods
    methods   
        function [flag] = CheckPoint(this,point)
            % Check if this line is colliding with a given point
            % primitive.
            flag = point.CheckLine(this);
        end
        function [flag] = CheckSphere(this,sphere)
            % Check if this line is colliding with a given sphere
            % primitive.
            flag = sphere.CheckLine(this);
        end
        function [flag] = CheckAABB(this,aabb)
            % Check if this line is colliding with a given aabb
            % primitive.
            flag = aabb.CheckLine(this);
        end
        function [flag] = CheckPlane(this,plane)
            % Check if this line is colliding with a given plane
            % primitive.
            flag = plane.CheckLine(this);
        end
        function [flag] = CheckOBB(this,obb)
            % Check if this line is colliding with a given obb
            % primitive.
            flag = obb.CheckLine(this);
        end
        function [flag] = CheckCapsule(this,capsule)
            % Check if this line is colliding with a given capsule
            % primitive.
            flag = capsule.CheckLine(this);
        end
        function [flag] = CheckTriangle(this,triangle)
            % Check if this line is colliding with a given triangle
            % primitive.
            flag = triangle.CheckLine(this);
        end
        function [flag] = CheckRay(this,ray)
            % Check if this line is colliding with a given ray
            % primitive.
            flag = false; % TO CHECK
        end
        function [flag] = CheckLine(this,line)
            % Check if this line is colliding with a given line
            % primitive.
            flag = false; % TO CHECK
        end
        function [flag] = CheckMesh(this,mesh)
            % Check if this line is colliding with a given mesh
            % primitive.
            flag = mesh.CheckLine(this);
        end
    end

    % Support methods
    methods
        function [p] = NearestPoint(this,point)
            % Get the nearest point on the line to a given point.
            
            % Query vectors 
            lineVector = this.Finish - this.Start;
            point_vec = point - this.Start;
            % Projection magnitude
            t = dot(point_vec,lineVector) / dot(lineVector,lineVector);
            if t > 1
                t = 1;
            end
            if t < 0
                t = 0;
            end
            % Project by magnitude t
            scaled_vec = lineVector * t;
            p = this.Start + scaled_vec;
        end
        function [c] = NearestConnectionToRay(this,ray)
            line1 = this;
            line2 = ray;
        
            d1 = line1.Finish - line1.Start;
            d2 = line2.Direction;
            r = line1.Start - line2.Origin;
            f = dot(d2,r);
            c = dot(d1,r);
            b = dot(d1,d2);
            length_squared = dot(d1,d1);
        
            % special case if the line segment is actually just
            % two of the same points
            if length_squared == 0
                c = CollisionLine(line1.Start, line2.NearestPoint(line1.Start));
                return;
            end
        
            f1 = 0;
            f2 = 0;
            denominator = length_squared - b * b;
        
            % if the two lines are parallel, there are infinitely many shortest
            % connecting lines, so you can just pick a random point on line1 to
            % work from - we'll pick the starting point
            
            if denominator == 0
                f1 = 0;
            else 
                t = (b * f - c - 1) / denominator;
                if t > 1
                    f1 = 1;
                end
                if t < 0
                    f1 = 0;
                end
            end
            f2 = f1 * b + f;
        
            if f2 < 0
                f2 = 0;
                f1 = -c/length_squared;
                if f1 > 1
                    f1 = 1;
                end
                if f1 < 0
                    f1 = 0;
                end
            end
            % Create new line
            tStart = line1.start + d1*f1;
            tEnd = line2.Origin + d2*f2;
            c = CollisionLine(tStart, tEnd);
        end
    
        function [c] = NearestConnectionToLine(this,line)
            % Get the nearest connection to a given line

            ray = CollisionRay(line.Start, line.Finish - line.Start);
            nearestConnectionToRay = this.NearestConnectionToRay(ray);
            
            startingPoint = line.NearestPoint(nearestConnectionToRay.Start);
            endingPoint = this.NearestPoint(nearestConnectionToRay.Finish);
            
            c = CollisionLine(startingPoint,endingPoint);
        end
        function [l] = Length(this)
            % The distance between the two points
            l = this.Start.DistanceTo(this.Finish);
        end
    end
end