classdef World < Module
    % World primitive is responsible for managing the hierarchical
    % relationships of transforms within the simulated world.

    properties (Constant)
        Name = "World Object";
    end
    properties (SetAccess = private)
        Root = Entity.empty;
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
            this.Root = Entity("Root");
            this.Root.Transform.IsStatic = true;
            this.WorldSize = worldSize;
        end
        % Get/Sets
        function set.Root(this,root)
            assert(isa(root,"Entity"),"Expecting a valid Entity reference.");
            this.Root = root;
        end
    end

    %% Interactions
    methods 
        function [this] = AddTransform(this,transform)
            % Add a transform to the world structure. 
            % If this transform is a disconnected tree, we must find the
            % root first, if the transform is a singleton, it is added
            % directly.
            assert(isa(transform,"Transform"),"Expecting a valid Transform reference.");
            
            rogueRoot = transform.GetRoot();
            if rogueRoot ~= this.Root.Transform
                rogueRoot.Parent = this.Root.Transform;
            end

        end
        function [this] = RemoveTransform(this,transform)
            % Add a transform to the world structure. 
            assert(isa(transform,"Transform"),"Expecting a valid Transform reference.");
            transform.RemoveParent();
        end
        function [this] = UpdateTransforms(this)
            % This function moves through the world hierarchy to compute
            % the transformations of all entities in the scene/world.

            % Move through hierarchically
            this.RecursiveTransformUpdate(this.Root.Transform);
            % Reset transform change status for next frame
            this.RecursiveNullifyChanges(this.Root.Transform);
        end
    end

    %% Internals
    methods (Static, Access = private)
        function RecursiveNullifyChanges(current)
            % Moves through the transform set and resets the change 

            % Set change status to negative
            current.Local.HasChanged = false;
            current.Inertial.HasChanged = false;

            % Has children? no? return
            if current.NumberOfChildren == 0
                return;
            end

            % Repeat for all children
            for ind = 1:current.NumberOfChildren
                % Get the child
                child = current.SelectChild(ind);
                % Recurse
                World.RecursiveNullifyChanges(child);
            end
        end
        function RecursiveTransformUpdate(current)
            % This method recursively moves throughout the scene to
            % recalculate the entity transformations.

            % Recompute the transformation properties
            [current] = World.UpdateTargetTransform(current);

            % Has children? no? return
            if current.NumberOfChildren == 0
                return;
            end

            % Repeat for all children
            for ind = 1:current.NumberOfChildren
                % Get the child
                child = current.SelectChild(ind);
                % Recurse
                World.RecursiveTransformUpdate(child);
            end
        end
        function [transform] = UpdateTargetTransform(transform)
            % This function updates the target transform to align its
            % inertial and local properties depending on what properties
            % have been changed during the frame.

            if transform.Local.HasChanged && transform.Inertial.HasChanged
                warning("Local and Inertia properties of Transform '%s' " + ...
                    "assigned in the same frame, unclear to do.", ...
                    transform.Entity.Name)
            end

            if transform.Local.HasChanged 
                % The change is a local one, recalculate inertial
                T = transform.Local.GetMatrix();
                if transform.NumberOfParents ~= 0
                    % Get the parent inertial transfrom
                    Tp = transform.Parent.Inertial.GetMatrix();
                    % Integrate
                    T = Tp*T;
                end
                % Assign the new inertial matrix
                transform.Inertial.SetMatrix(T);
            end

            if transform.Inertial.HasChanged 
                % The change is a global one, recalculate local
                T = transform.Inertial.GetMatrix();
                if transform.NumberOfParents ~= 0
                    % Get the parent inertial transfrom
                    Tp = transform.Parent.Inertial.GetMatrix();
                    % Get the difference between them 
                    T = T/Tp;
                end
                % Assign the new local matrix
                transform.Local.SetMatrix(T);
            end

        end
    end
end