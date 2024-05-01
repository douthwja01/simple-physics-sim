classdef (Abstract) World < handle
    % World primitive is responsible for managing the hierarchical
    % relationships of transforms within the simulated world.

    properties (SetAccess = private)
        Root = Transform();
        WorldSize = 10;
    end
    % Main
    methods
        function [this] = World(worldSize)
            % WORLD - Construct an instance of the world object.

            % Input check
            if nargin < 1
                worldSize = 10;
            end

            % Parameterize
            this.Root = Transform();
            this.WorldSize = worldSize;
        end
        % Get/Sets
        function set.Root(this,transform)
            assert(isa(transform,"Transform"),"Expecting a valid Transform reference.");
            this.Root = transform;
        end
    end
    methods %(Access = protected)
        function [this] = AddTransform(this,transform)
            % Add a transform to the world structure. 
            % If this transform is a disconnected tree, we must find the
            % root first, if the transform is a singleton, it is added
            % directly.
            assert(isa(transform,"Transform"),"Expecting a valid Transform reference.");
            
            rogueRoot = transform.GetRoot();
            if rogueRoot ~= this.Root
                rogueRoot.Parent = this.Root;
            end

        end
        function [this] = RemoveTransform(this,transform)
            % Add a transform to the world structure. 
            assert(isa(transform,"Transform"),"Expecting a valid Transform reference.");
            transform.RemoveParent();
        end
    end
end