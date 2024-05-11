classdef MeshExtensions
    %MESHGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function [mesh] = UnitPlane()
            % This function creates a simple unit plane
            
            % Simple unit vertices
            vertices = [-0.5,0.5,0;0.5,0.5,0;0.5,-0.5,0;-0.5,-0.5,0];
            % Simple plane faces
            faces = [1,2,3;1,3,4];
            % convert to mesh
            mesh = Mesh(vertices,faces);
        end
        function [mesh] = UnitSphere()
            % This function creates a unit sphere centered around zero.
            mesh = MeshExtensions.Sphere(zeros(3,1),1);
        end
        function [mesh] = UnitCube()
            % This function creates a unit cube centered around zero.
            unitVertices = 0.5*ones(3,1);
            mesh = MeshExtensions.CuboidFromExtents(-unitVertices,unitVertices);
        end
    end

    methods (Static)
        % Mass marker 
        function [major,minor] = MassMarker(center,scale)
            % This function will create a 3D "center of mass" marker mesh.
            
            % Credit given to the original author: 
            % https://uk.mathworks.com/matlabcentral/fileexchange/34851-plot-a-center-of-mass
            
            % Sanity check
            if nargin < 1
                scale = 1;
            end
            k = "smooth";
%             k = "coarse";
            switch k
                case 'coarse'
                    a=.44433292853227944784221559874174;
                    % Spherical vertices
                    v = [ 0 0         1;
                        sin(pi/8) 0 cos(pi/8);0         sin(pi/8) cos(pi/8);
                        sin(pi/4) 0 sin(pi/4);a a sqrt(1-2*a^2);0 sin(pi/4) sin(pi/4);
                        cos(pi/8) 0 sin(pi/8);sqrt(1-2*a^2) a a;a sqrt(1-2*a^2) a;0 cos(pi/8) sin(pi/8);
                        1 0 0;cos(pi/8) sin(pi/8) 0;sin(pi/4) sin(pi/4) 0;sin(pi/8) cos(pi/8) 0;0 1 0]./50;
                    % Generate face connectivity matrix                  
                    f  = [1 2 3;2 4 5;2 5 3;
                        3 5 6;4 7 8;4 8 5;
                        5 8 9;5 9 6;6 9 10;
                        7 11 12;7 12 8;8 12 13;
                        8 13 9;9 13 14;9 14 10;
                        10 14 15];
                    f1 = [f;f+15;f+30;f+45];
                case 'smooth'
                    % smoother com: what is the correct value for b?
                    % b=5275338780780258*2^(-54);
                    b  = 5271341053858645*2^(-54);
                    c  = 5020924469873901*2^(-53);
                    e  = 5590528873889244*2^(-54);
                    a  = [sin(pi/12) cos(pi/12)  sin(pi/6) cos(pi/6) sqrt(.5) sqrt(1-2*b^2) sqrt(1-c^2-e^2)];
                    % Spherical vertices
                    v  = [0 0 1;a(1) 0 a(2);0 a(1) a(2);a(3) 0 a(4);b b a(6);0 a(3) a(4);
                        a(5) 0 a(5);c e a(7);e c a(7);0 a(5) a(5);a(4) 0 a(3) ;a(7) e c;[1 1 1]/sqrt(3);
                        e a(7) c;0 a(4) a(3);a(2) 0 a(1) ;a(6) b b;a(7) c e;c a(7) e;b a(6) b;0 a(2) a(1);
                        1 0 0;a(2) a(1) 0;a(4) a(3) 0;a(5) a(5) 0;a(3) a(4) 0;a(1) a(2) 0;0 1 0];
                    % Generate face connectivity matrix
                    f  = [1 2 3;2 4 5;2 5 3;3 5 6;4 7 8;4 8 5;5 8 9;5 9 6;6 9 10;7 11 12;7 12 8;8 12 13;8 13 9;9 13 14;
                        9 14 10;10 14 15;11 16 17;11 17 12;12 17 18;12 18 13;13 18 19;13 19 14;14 19 20;14 20 15;15 20 21;
                        16 22 23;16 23 17;17 23 24;17 24 18;18 24 25;18 25 19;19 25 26;19 26 20;20 26 27;20 27 21;21 27 28];
                    f1 = [f;f+28;f+56;f+84];
            end
            % Align vertex permutations
            v0 = [v;[-v(:,1) -v(:,2) v(:,3)];[ v(:,1) -v(:,2) -v(:,3)];[-v(:,1) v(:,2) -v(:,3)]];
            % Transform vertex pair
            Ts = Transform.Scale(scale);
            padding = ones(size(v0,1),1);
            % Create the two component meshes
            major = Mesh([ v0,padding]*Ts + center',f1);
            minor = Mesh([-v0,padding]*Ts + center',f2);
        end
        function [mesh] = Cylinder(p0,p,radius)
            % This function generates a mesh as a cylinder between two
            % points with a defined radius
            
            % Sanity check
            assert(IsColumn(p0,3) && IsColumn(p,3),"Expecting two Cartesian points defining the axis [3x1]");
            assert(iscalar(radius),"Expecting a scalar cylinder radius.");
            
            % Assemble the cylinder geometry
            axis = p - p0;
            unitAxis = axis/norm(axis);
            perpVector = cross(unitAxis,[1;0;0]);
            if sum(perpVector) == 0
                perpVector = cross(unitAxis,[0;1;0]);
            end
            perpVector = radius*perpVector;               
            points = 10;
            nodalAngle = pi/2;
            increment = 2*pi/points;
            coordinates = zeros(2*points,3);
            for i = 1:points
                % Calculate the vector as a result of the perpendicular
                % rotated through "nodal angle"
                radialVector = RodriguesRotation(perpVector,unitAxis,nodalAngle);
                coordinates(i,:) = (p0 + radialVector)';       % Ring defining base
                coordinates(i+10,:) = (p + radialVector)';     % Ring defining top
                % Iterate
                nodalAngle = nodalAngle - increment;
            end
            % Triangulate rotated volume
            [faces,~] = convhull(coordinates(:,1),coordinates(:,2),coordinates(:,3),"simplify",true);
            % Generate the mesh object
            mesh = Mesh(vertices,faces);
        end
        function [mesh] = Triangle(p0,p1,p2)
            % This function generates a triangular mesh from a set of
            % points.

            % Insert the coordinates directly
            vertices = zeros(3);
            vertices(1,:) = p0';
            vertices(2,:) = p1';
            vertices(3,:) = p2';
            % The connectivity
            faces = [1;2;3];
            % Generate the mesh object
            mesh = Mesh(vertices,faces);
        end
        function [mesh] = Plane(center,normal,width,height)
            % Generates a planar mesh at a given position, with a provided
            % normal.

            % Sanity check
            assert(IsColumn(center,3),"Expecting a position coordinate [3x1].");
            assert(IsColumn(normal,3),"Expecting a 3D normal vector [3x1].");
            assert(isscalar(width),"Expecting a scalar width.");
            assert(isscalar(height),"Expecting a scalar height.");

            % Scalar extents
            unitWidth = width/2;
            unitHeight = height/2;

            % Map the extents to coordinates
            topLeft     = [-unitWidth;unitHeight;0];
            bottomLeft  = [-unitWidth;-unitHeight;0];
            topRight    = [unitWidth;unitHeight;0];
            bottomRight = [unitWidth;-unitHeight;0];
            % Create the vertices
            vertices = zeros(4,3);
            vertices(1,:) = (center + topRight)'; 
            vertices(2,:) = (center + bottomRight)';
            vertices(3,:) = (center + topLeft)';
            vertices(4,:) = (center + bottomLeft)';
            % Assign the faces
            faces = [3,1,2;3,4,2];
            % Create the mesh object
            mesh = Mesh(vertices,faces);      
        end
        
        function [mesh] = CuboidFromRadius(radius)
            % This function creates a set of vertices for a cube
            % encapsuated in a sphere of a given radius.

            % Calculate extents from radius
            v = ones(3,1)*radius/1.7321;                                 	% Rate of dimensional expansion
            % Generate the equivalent extent-defined cuboid
            mesh = MeshExtensions.CuboidFromExtents(-v,v);
        end
        function [mesh] = CuboidFromExtents(minExtents,maxExtents)
            % Return a matrix of point defining a cuboid scaled to that of
            % a dimensions provided.
            
            % Sanity check
            assert(numel(minExtents) == 3,"Expecting two vectors defining min:max in each dimension");
            assert(numel(maxExtents) == 3,"Expecting a column vector of dimension maximums [3x1].")
            
            % Define vertex data from limits
            vertices = zeros(8,3);
            vertices(1,:) = [maxExtents(1),maxExtents(2),maxExtents(3)];
            vertices(2,:) = [maxExtents(1),maxExtents(2),minExtents(3)];
            vertices(3,:) = [maxExtents(1),minExtents(2),minExtents(3)];
            vertices(4,:) = [maxExtents(1),minExtents(2),maxExtents(3)];
            vertices(5,:) = [minExtents(1),minExtents(2),minExtents(3)];
            vertices(6,:) = [minExtents(1),minExtents(2),maxExtents(3)];
            vertices(7,:) = [minExtents(1),maxExtents(2),maxExtents(3)];
            vertices(8,:) = [minExtents(1),maxExtents(2),minExtents(3)];
            % Define face connectivity matrix
            faces =  [ 1,2,7; 1,4,2; 1,7,4; 2,3,8; 2,4,3; 2,8,7; 3,4,6;
                       3,5,8; 3,6,5; 4,7,6; 5,6,8; 6,7,8]; 
            % Create a new mesh object
            mesh = Mesh(vertices,faces);
        end
        function [mesh] = Sphere(center,radius,faceNumber)
            % This function returns a sphere coordinate cloud at a given
            % position in space, of a given radius.
            
            % Ensure valid axes            
            if nargin < 3
                faceNumber = 10;
            end

            % Sanity check
            assert(IsColumn(center,3),"Expecting a cartesian position [3x1].");
            assert(isscalar(radius),"Expecting a scalar radius.");

            % Triangulate a sphere
            [X,Y,Z] = sphere(faceNumber);
            X = center(1) + X.*radius;
            Y = center(2) + Y.*radius;
            Z = center(3) + Z.*radius;
            % Generate patch object
            [faces,vertices,~] = surf2patch(X,Y,Z,'triangles');
            % Create a mesh object
            mesh = Mesh(vertices,faces);
        end
    end
end

