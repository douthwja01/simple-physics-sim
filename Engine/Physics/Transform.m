classdef Transform < TreeElement
    %TRANSFORM The geometric representation of the entity in 3D space.

    properties (Hidden)
        %         PriorPosition = [];
        Local = SO3.empty;
        %Inertial = SO3.empty;
    end
    % Main
    methods
        function [this] = Transform(entity)
            %TRANSFORM Construct an instance of the pose-element
            %   Detailed explanation goes here

            if nargin < 1
                entity = Entity.empty;
            end

            % Assign the entity
            [this] = this@TreeElement(entity);

            % Create a new transform object
            this.Local = SO3.Zero;
            %this.Inertial = SO3.Zero;
        end
        % Get/sets for local properties
        function [p] = GetPosition(this)
            % Get the position of this transform in the parent space.
            p = this.Local.Position;
        end        
        function [this] = SetPosition(this,p)
            % Set the position of this transform in the parent space.
            this.Local.Position = p;
        end
        function [p] = GetOrientation(this)
            % Get the local rotation from the transform.
            p = this.Local.Orientation;
        end        
        function [this] = SetOrientation(this,q)
            % Set the orientation of this transform in the parent space.
            this.Local.Orientation = q;
        end
        function [T] = GetMatrix(this)
            % Get the transformation matrix of this transform in the parent
            % space.
            T = this.Local.ToMatrix();
        end
        function [this] = SetMatrix(this,T)
            % Set the local transformation matrix within the parent space
            % (the local transformation).
            this.Local = SO3.FromMatrix(T);
            %this.HasChanged = true;
        end
        function [Ts] = GetScaleMatrix(this)
            % Get scale matrix of this transform in the parent space (the 
            % local scale matrix).
            Ts = this.Local.ToScaleMatrix();
        end
        function [s] = GetScale(this)
            % Get dimensional scalars of this transform in the parent space
            % (local scalars).
            s = this.Local.Scale;
        end
        function [this] = SetScale(this,s)
            % Set dimensional scalars of this transform in the parent
            % space (local scalars).
            this.Local.Scale = s;
            this.HasChanged = true;
        end
        % Get/sets for parent properites
        function [p] = GetPositionParentSpace(this)
            % Get the position of this transform in the parent space.
            p = this.Local.Position;
        end       
        function [q] = GetOrientationParentSpace(this)
            % Get the orientation of this transform in the parent space.
            q = this.Local.Orientation;
        end        
        % Get/sets for world properties
        function [T] = GetWorldTransform(this)
            % Get the transformation of this matrix in the world space.

            T = this.Local.transform;
            if this.NumberOfParents == 0
                return
            end
            T = this.Parent.GetWorldTransform()*T;
        end
        function [p] = GetWorldPosition(this)
            % Get the position of this transform in the world space.

            if this.NumberOfParent == 0
                p = this.Local.Position;
            else
                worldOrientation = this.Parent.GetWorldOrientation();
                worldOffset =  this.Parent.GetWorldPosition();
                p = QuatTransform(worldOrientation,this.Local.Position) + worldOffset;
            end
        end
        function [q] = GetWorldOrientation(this)
            % Get the orientation of the transform in the world space.
            if this.NumberOfParents == 0
                q = this.Local.Orientation;
            else
                q = QuatMultiply(this.Parent.GetWorldOrientation(), this.Local.Orientation);
            end
        end
    end
    % Utilities
    methods
        function [v_l] = InverseTransformVector(this,v_w)
            % Transform a vector in the world frame into the local frame of
            % this transform.

            % Get the transformation matrix in the world
%             Rwl = this.Inertial.GetRotationMatrix();
            Twl = this.GetWorldTransform();

            % Multiply by the matrix
            v_l = Twl'*[v_w;0];
            v_l = v_l(1:3,1);
        end
        function [v_w] = TransformVector(this,v_l)
            % Transform a vector from this transform frame into the world
            % frame.

            % Get the transformation matrix in the world
            %Rwl = this.Inertial.GetRotationMatrix();
            Twl = this.GetWorldTransform();
            % Multiply by the matrix
            v_w = Twl*[v_l;0];
            v_w = v_w(1:3,1);
        end
        function [p_l] = InverseTransformPoint(this,p_w)
            % Transform a point in the world frame into the local frame of
            % this transform.

            % Get the transformation matrix in the world
            %Twl = this.Inertial.GetMatrix();
            Twl = this.GetWorldTransform();
            % Invert the matrix and multiply
            p_l = Twl\[p_w;1];
            % Remove the loose element
            p_l = p_l(1:3,1);
        end
        function [p_w] = TransformPoint(this,p_l)
            % Transform a point in this transform frame into the world frame.

            % Get the transformation matrix in the world
%             Twl = this.Inertial.GetMatrix();
            Twl = this.GetWorldTransform();
            % Multiply the matrix
            p_w = Twl*[p_l;1];
            % Remove the loose element
            p_w = p_w(1:3,1);
        end
    end
    % Transformations
    methods
        function [p] = WorldToLocalSpace(this,p_world)
            % Remap a point provided in the world space to the local space
            % of this transform.

            if this.NumberOfParents == 0
                p = p_world;
            else
                p = this.Parent.WorldToLocalSpace(p_world);
            end
            p = QuatTransformInverse(this.Local.Orientation, p - this.Local.Position);
        end
        function [p] = WorldToParentSpace(this,p_world)
            % Remap a point provided in the world space to the parent space
            % of this transform.

            p_local = this.WorldToLocalSpace(p_world);
            p = this.LocalToParentSpace(p_local);
        end
        function [p] = LocalToWorldSpace(this,p_local)
            % Remap a point provided in the local space to the world space
            % of this transform.

            p = QuatTransform(this.Local.Orientation, p_local);
            d = p + this.Local.Position;
            if this.NumberOfParents == 0
                p = d;
            else
                p = this.Parent.LocalToWorldSpace(d);
            end
        end
        function [p] = LocalToParentSpace(this,p_local)
            % Remap a point provided in the local space to the parent space
            % of this transform.

            p_world = this.LocalToWorldSpace(p_local);
            if this.NumberOfParents == 0
                p = p_world;
            else
                p = this.Parent.WorldToLocalSpace(p_world);
            end
        end
        function [d] = WorldToLocalDirection(this,d_world)
            % Remap a direction provided in the world space to the local
            % space of this transform.

            worldOrientation = this.GetWorldOrientation();
            d = QuatTransformInverse(worldOrientation, d_world);
        end
        function [d] = WorldToParentDirection(this,d_world)
            % Remap a direction provided in the world space to the parent
            % space of this transform.

            d_local = this.WorldToLocalDirection(d_world);
            d = this.LocalToParentDirection(d_local);
        end
        function [d] = LocalToWorldDirection(this,d_local)
            % Remap a direction provided in the local space to the world
            % space of this transform.

            worldOrientation = this.GetWorldOrientation();
            d = QuatTransform(worldOrientation, d_local);
        end
        function [d] = LocalToParentDirection(this,d_local)
            % Remap a direction provided in the local space to the parent
            % space of this transform.

            p_world = this.LocalToWorldDirection(d_local);
            if this.NumberOfParents == 0
                d = p_world;
            end
            d = this.Parent.WorldToLocalDirection(p_world);
        end
        function [p] = ParentToLocalSpace(this,p)
            % Remap a point provided in the parent space to the local space
            % of this transform.

            if this.NumberOfParents == 0
                d = p;
            else
                d = this.ParentToWorldSpace(p);
            end
            p = WorldToLocalSpace(d);
        end
        function [p] = ParentToWorldSpace(this,p)
            % Remap a point provided in the parent space to the world space.

            if this.NumberOfParents == 0
                return
            end
            p = this.Parent.LocalToWorldSpace(p);
        end
        function [d] = ParentToLocalDirection(this,p)
            % Remap a direction provided in the parent space to the local
            % space of this transform.

            if this.NumberOfParents == 0
                d = p;
            else
                d = this.ParentToWorldDirection(p);
            end
            d = this.WorldToLocalDirection(d);
        end
        function [p] = ParentToWorldDirection(this,p)
            % Remap a direction provided in the parent space to the world
            % space.

            if this.NumberOfParents == 0
                return
            end
            p = this.Parent.LocalToWorldDirection(p);
        end
        function [q] = WorldToLocalOrientation(this,q)
            % Remap an orientation in the world to a orientation in the
            % local space of this transform.

            q_world = this.GetWorldOrientation();
            q_world = Normalize(q_world);
            q_inv   = QuatInvert(q_world);
            q = QuatMultiply(q_inv, q);
        end
        function [q] = WorldToParentOrientation(this,q)
            % Remap an orientation in the world to an orientation in the
            % parent space of this transform.

            if this.NumberOfParents == 0
                return
            end
            q_parent = this.Parent.GetWorldOrientation();
            q_parent = Normalize(q_parent);
            q_inv = QuatInvert(q_parent);
            q = QuatMultiply(q_inv,q);
        end
    end
    % Visualisation
    methods
        function [h] = Plot(this,container)
            % Plot the transform
            if nargin < 2
                container = gca;
            end
            % Plot transform
            h = this.Inertial.Plot(container);
        end
    end
end

