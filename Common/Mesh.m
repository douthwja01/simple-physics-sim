
classdef Mesh < handle
    % A simple container primitive to hold mesh vertices and faces prior to
    % matlab's graphical utilities.

    % Assumes that the vertices are centered around the origin.

    properties
        Vertices = [];
        Faces = [];
    end
    properties (Dependent)
        NumberOfVertices;
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
        end
        function set.Faces(this,f)
            assert(size(f,2) == 3,"Expecting an array of face-vertex indices.");
            this.Faces = f;
        end
        function [n] = get.NumberOfVertices(this)
            n = size(this.Vertices,1);
        end
    end
    % Utilities
    methods
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
        function [h] = Draw(this,container)
            % Draw this mesh to a given graphical container
            if nargin < 2
                container = gca;
            end
            % Generate patch
            h = patch(container,'Vertices',this.Vertices,'Faces',this.Faces);        
        end
    end
end