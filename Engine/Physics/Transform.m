classdef Transform < TreeElement
    %TRANSFORM The geometric representation of the entity in 3D space.
    
    % Kinematic properties (simple containers)    
    properties (Dependent)
        Position;
        Rotation;
        Scale;
    end
    properties
        % Kinematics
        Velocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
        Acceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
        IsStatic = false;
    end

    properties (Hidden)
        PriorPosition = [];
        Pose = SO3.empty;
    end

    methods
        function [this] = Transform(entity)
            %POSE Construct an instance of the pose-element
            %   Detailed explanation goes here

            if nargin < 1
                entity = Entity.empty;
            end

            % Assign the entity
            [this] = this@TreeElement(entity);

            % Create a new transform object
            this.Pose = SO3.Zero;
        end
        % Get/sets
        function [p] = get.Position(this)
            p = this.Pose.Position;
        end
        function set.Position(this,p)
            this.Pose.Position = p;
        end
        function [q] = get.Rotation(this)
            q = this.Pose.Rotation;
        end
        function set.Rotation(this,q)
            this.Pose.Rotation = q;
        end
        function [s] = get.Scale(this)
            s = this.Pose.Scale;
        end
        function set.Scale(this,s)
            this.Pose.Scale = s;
        end
        % Kinematics (should be somewhere else?)
        function set.Velocity(this,v)
            assert(IsColumn(v,3),"Expecting a valid Cartesian linear velocity [3x1].");
            this.Velocity = v;
        end
        function set.AngularVelocity(this,w)
            assert(IsColumn(w,3),"Expecting a valid Cartesian angular velocity [3x1].");
            this.AngularVelocity = w;
        end 
        function set.Acceleration(this,dv)
            assert(IsColumn(dv,3),"Expecting a valid Cartesian linear acceleration [3x1].");
            this.Acceleration = dv;
        end  
        function set.AngularAcceleration(this,dw)
            assert(IsColumn(dw,3),"Expecting a valid Cartesian angular acceleration [3x1].");
            this.AngularAcceleration = dw;
        end 
        function set.IsStatic(this,s)
            assert(islogical(s),"Expecting a valid logical IsStatic flag.");
            this.IsStatic = s;
        end
    end

    % Utilities
    methods
        function [v_l] = InverseTransformVector(this,v_w)
            % Transform a vector in the world frame into the local frame of
            % this transform.

            % Get the transformation matrix in the world
            Rwl = this.GetRotationMatrix();
            % Multiply by the matrix
            v_l = Rwl'*v_w;
        end
        function [v_w] = TransformVector(this,v_l)
            % Transform a vector from this transform frame into the world
            % frame.

            % Get the transformation matrix in the world
            Rwl = this.GetRotationMatrix();
            % Multiply by the matrix
            v_w = Rwl*v_l;
        end
        function [p_l] = InverseTransformPoint(this,p_w)
            % Transform a point in the world frame into the local frame of
            % this transform.

            % Get the transformation matrix in the world
            Twl = this.GetWorldMatrix();
            % Invert the matrix and multiply
            p_l = transpose(Twl)*[p_w;1];
            % Remove the loose element
            p_l = p_l(1:3,1);
        end
        function [p_w] = TransformPoint(this,p_l)
            % Transform a point in this transform frame into the world frame.

            % Get the transformation matrix in the world
            Twl = this.GetWorldMatrix();
            % Multiply the matrix
            p_w = Twl*[p_l;1];
            % Remove the loose element
            p_w = p_w(1:3,1);
        end
    end

    % (World) representation
    methods
        function [T] = GetWorldMatrix(this)
            % Get the world transformation matrix.

            % Get the local matrix
            T = this.GetLocalMatrix();
            if this.NumberOfParents == 0
                return
            end
            % Get the world transformation of the parent
            Tp = this.Parent.GetWorldMatrix();
            % Multiply by the parent matrix
            T = Tp*T;
        end
        function [this] = SetWorldMatrix(this,Tw)
            % Set the World transformation matrix.

            % Get the local matrix
            if this.NumberOfParents == 0
                this.SetLocalMatrix(Tw);
                return
            end
            % Get the current world matrix
            Tw_inv = this.GetWorldMatrix;
            % Get the difference between them [To Check]
            Tl = Tw/Tw_inv;
            % [To fix after parentage]
            this.SetLocalMatrix(Tl);
        end
        function [p] = GetWorldPosition(this)
            % This transform multiplied by all its parents

            % Get the local matrix
            T = this.GetWorldMatrix();
            if this.NumberOfParents == 0
                p = this.Position;
                return
            end
            % Extract from the transform
            p = T(4,1:3)';
        end
        function [this] = SetWorldPosition(this,p)
            % Allow the setting of the transform's world position.

            %p_parent = this.Parent.GetWorldPosition();
            
            pLocal = this.InverseTransformPoint(p);

            % [To fix after parentage]
            this.SetLocalPosition(pLocal);
        end
        function [q] = GetWorldRotation(this)
            % return the current world-frame rotation

            ql = this.GetLocalRotation();
            if this.NumberOfParents == 0
                q = ql;
                return
            end
            % Get the parent rotation
            qParent = this.Parent.GetWorldRotation();
            % Multiple local with parent world
            q = qParent*q1;
        end
        function [this] = SetWorldRotation(this,q)
            % Allow setting of the rotation in the world frame.

            % Get the local matrix
            if this.NumberOfParents == 0
                this.SetLocalRotation(q);
                return
            end

            qParent = this.Parent.GetWorldRotation();
            % Subtract the parent rotation
            qLocal = q.Multiply(qParent);

            % Assign the local rotation
            this.SetLocalRotation(qLocal);
        end
        % World Scaling
        function [Ts] = GetWorldScaleMatrix(this)
            % Get the matrix representation of the scale

            % [To fix after parentage]
            Ts = this.GetLocalScaleMatrix();
        end
        function [s] = GetWorldScale(this)
            % Get the scale in the world-frame

            % [To fix after parentage]
            s = this.GetLocalScale();
        end
        function [this] = SetWorldScale(this,s)
            % Allow the setting of the world scale.

            % [To fix after parentage]
            this.SetLocalScale(s);
        end
    end
    % (Local) Representation
    methods
        function [T] = GetLocalMatrix(this)
            % Get the local transform matrix, this matrix already
            % contains the rotation,translation and scaling.
            T = this.Pose.ToMatrix();
        end
        function [this] = SetLocalMatrix(this,T)
            % Set the local transform matrix            
            this.Pose = SO3.FromMatrix(T);
        end
        function [p] = GetLocalPosition(this)
            % Get the local position from the transform.
            p = this.Pose.Position;
        end
        function [this] = SetLocalPosition(this,p)
            % Set the local position via the transform.
            this.Pose.Position = p;
        end
        function [p] = GetLocalRotation(this)
            % Get the local rotation from the transform.
            p = this.Pose.Rotation;
        end
        function [this] = SetLocalRotation(this,q)
            % Set the local rotation via the transform.
            this.Pose.Rotation = q;
        end
        % Scaling
        function [Ts] = GetLocalScaleMatrix(this)
            % Get the matrix representation of the scale
            Ts = this.Pose.ToScaleMatrix();
        end
        function [s] = GetLocalScale(this)
            % Get the local scale defined by this transform
            s = this.Pose.Scale;
        end
        function [this] = SetLocalScale(this,s)
            % Set the scale vector of the transform
            this.Pose.Scale = s;
        end
        % Visualisation
        function [h] = Plot(this,container)
            % Plot the transform
            if nargin < 2
                container = gca;
            end
            % Plot transform 
            h = this.Pose.Plot(container);
        end
    end
end

