
classdef Mesh < handle
    % A simple container primitive to hold mesh vertices and faces prior to
    % matlab's graphical utilities.

    % Assumes that the vertices are centered around the origin.

    properties
        Vertices = [];
        Faces = [];
        Normals = [];
    end
    properties (Dependent)
        NumberOfVertices;
        NumberOfFaces;
    end
    % Main
    methods
        function [this] = Mesh(vertices,faces)
            % CONSTRUCTOR - Allow the creation of a mesh object from an
            % array off vertices and faces.

            if nargin < 2
                return;
            end
            % Assign the initial properties
            this.Vertices = vertices;
            this.Faces = faces;
        end
        % Get/sets
        function set.Vertices(this,v)
            assert(size(v,2) == 3,"Expecting an array of vertex coordinates.");
            this.Vertices = v;
            this.Normals = this.CalculateNormals(this.Faces,this.Vertices);
        end
        function set.Faces(this,f)
            assert(size(f,2) == 3,"Expecting an array of face-vertex indices.");
            this.Faces = f;
            this.Normals = this.CalculateNormals(this.Faces,this.Vertices);
        end
        function [n] = get.NumberOfVertices(this)
            n = size(this.Vertices,1);
        end
        function [n] = get.NumberOfFaces(this)
            n = size(this.Faces,1);
        end
    end
    % Utilities
    methods
        function [point] = GetFarthestPointInDirection(this,vec)
            % Sanity check
            assert(IsColumn(vec,3),"Expecting a valid 3D vector points [3x1].");
            vec = vec/norm(vec);

            maxMag = 0;
            index = 1;
            for i = this.NumberOfVertices
                mag = dot(vec,this.Vertices(i));
                if (mag > maxMag)
                    maxMag = mag;
                    index = i;
                end
            end
            point = this.Vertices(index,:)';
        end
        function [mesh] = TransformBy(this,Tf)
            % This function returns this mesh transformed by a given
            % transform matrix.
            
            % Sanity check
            assert(size(Tf,1) == 4 && size(Tf,2) == 4,"Expecting a valid transformation matrix [4x4].");

            % Transform the vertices
            padding = ones(this.NumberOfVertices,1);
            modifiedVertices = Tf*[this.Vertices,padding]';
            modifiedVertices = modifiedVertices(1:3,:)';

            % Create the two component meshes
            mesh = Mesh(modifiedVertices,this.Faces);
        end
        function [mesh] = ScaleBy(this,width,depth,height)
            % Scale the mesh by a set of dimensional values.
            tf = eye(4)*[width,depth,height;0];
            mesh = this.TransformBy(tf);
        end
        function [h] = Draw(this,container)
            % Draw this mesh to a given graphical container
            if nargin < 2
                container = gca;
            end
            % Generate patch
            h = patch(container,'Vertices',this.Vertices,'Faces',this.Faces);        
        end
    end
    methods (Static,Access = private)
        function [normals] = CalculateNormals(f,v)
            % This function computes the normals of all the mesh faces.

            % Sanity check
            if size(f,2) ~= 3 || size(v,2) ~= 3
                normals = [];
                return
            end

            % Calculate the normals
            normals= zeros(size(f,1),3);
            for i = 1:size(f,1)
                face = f(i,:);
                points = v(face,:);
                % Calculate the point faces
                normals(i,:) = TripleProduct(points(1,:),points(2,:),points(3,:));
            end
        end
    end
end