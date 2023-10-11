
classdef Mesh < handle
    % A simple container primitive to hold mesh vertices and faces prior to
    % matlab's graphical utilities.

    properties
        Vertices = [];
        Faces = [];
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
    end
    % Utilities
    methods
        function [mesh] = TransformBy(this,Tf)
            % This function returns this mesh transformed by a given
            % transform matrix.
            
            % Sanity check
            assert(size(Tf,1) == 4 && size(Tf,2),"Expecting a valid transformation matrix [4x4].");

            % Transform the vertices
            padding = ones(size(this.Vertices,1),1);
            modifiedVertices = [ this.Vertices,padding]*Tf;
            modifiedVertices = modifiedVertices(1:3,:);
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