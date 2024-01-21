
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
            this.UpdateNormals();
        end
        function set.Faces(this,f)
            assert(size(f,2) == 3,"Expecting an array of face-vertex indices.");
            this.Faces = f;
            this.UpdateNormals();
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
        function [mesh] = ScaleBy(this,x,y,z)
            % Scale the mesh by a set of dimensional values.
            tf = diag([x;y;z;0]);
            mesh = this.TransformBy(tf);
        end
        function [h] = Draw(this,container,colour)
            % Draw this mesh to a given graphical container
            if nargin < 3
                colour = 'b';
            end
            if nargin < 2
                container = gca;
            end
            % Generate patch
            h = patch(container,'Vertices',this.Vertices,'Faces',this.Faces);        
            set(h,"FaceColor",colour);
        end
    end
    methods (Access = private)
        function [this] = UpdateNormals(this)
            % This function computes the normals of all the mesh faces.

            % Sanity check
            if size(this.Faces,2) ~= 3 || size(this.Vertices,2) ~= 3
                return
            end

            % Calculate the normals
            normals = zeros(size(this.Faces,1),3);
            for i = 1:size(this.Faces,1)
                face = this.Faces(i,:);
                points = this.Vertices(face,:);
                % Calculate the point faces
                normals(i,:) = TripleProduct(points(1,:),points(2,:),points(3,:));
            end
            this.Normals = normals;
        end
    end
    % Mesh Tools
    methods (Static)
        function [mesh] = EnclosingCube(mesh)
            % This function generates an cuboid mesh representing the
            % min-max's of the provided mesh (a minimal enclosing volume).

            [xlim,ylim,zlim] = Mesh.Extents(mesh);
            % Reformat
            minimums = [xlim(1);ylim(1);zlim(1)];
            maximums = [xlim(2);ylim(2);zlim(2)];
            % Generate the cuboid reference mesh
            mesh = MeshGenerator.CuboidFromExtents(minimums,maximums);
        end
        function [xlim,ylim,zlim] = Extents(mesh)
            % This function simply extracts the dimensional min-max's from
            % the arbitrarily complex mesh.

            assert(isa(mesh,"Mesh"),"Expecting a valid mesh")

            % Extract the dimensional limits
            v = mesh.Vertices;
            xlim = [min(v(:,1)),max(v(:,1))];
            ylim = [min(v(:,2)),max(v(:,2))];
            zlim = [min(v(:,3)),max(v(:,3))];
        end
    end
end