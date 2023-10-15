
classdef Ray 
    % A simple ray primitive
    
    properties
        Direction = [1;0;0];
        Origin = zeros(3,1);
        Magnitude = 1;
    end
    % Main
    methods
        % DEFINE RAY
        function [this] = Ray(origin,direction,magnitude)
            % Define a ray with a given a origin and direction.
            assert(IsColumn(origin,3) && IsColumn(direction,3),'Parameters must be 3D');
            
            if nargin < 3
                magnitude = 1;
            end
            % Set the properties
            this.Origin = origin;
            this.Direction = direction/norm(direction);
            this.Magnitude = magnitude;
        end
    end
    % Utilities
    methods (Static)
        function [r] = FromPoints(a,b)
            % This function creates a ray between two points with a
            % magnitude of their seperation.
            assert(IsColumn(a,3),"Expecting a 3D first point.");
            assert(IsColumn(b,3),"Expecting a 3D second point.");
            % Create the ray
            r = Ray.FromVector(a,b-a);
        end
        function [r] = FromVector(origin,vector)
            assert(IsColumn(origin,3),"Expecting a 3D coordinate.");
            assert(IsColumn(vector,3),"Expecting a 3D coordinate.");
            % Create the ray
            vn = norm(vector);
            unit = vector/vn;
            r = Ray(origin,unit,vn);
        end
        function [d] = PointProjection(ray,point)
            % This function calculates the projection of a point on a ray.
            assert(isa(ray,"Ray"),"Expecting a valid ray.");
            assert(IsColumn(point,3),"Expecting a 3D coordinate.");
            % Calcuate the projection magnitude(distance)
            d = dot(ray.Direction,(point - ray.Origin));
        end
        function [p] = ProjectDistance(ray,d)
            % This function returns a point at distance d along a ray.
            assert(isa(ray,"Ray"),"Expecting a valid ray.");
            assert(isscalar(d),"Expecting scalar distance.");
            % Calculate the point
            p = ray.Origin + d*ray.Direction;
        end
    end
end