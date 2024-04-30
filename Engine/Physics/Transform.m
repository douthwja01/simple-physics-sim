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
        SO3 = SO3.empty;
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
            this.SO3 = SO3.Zero;
        end
        % Get/sets
        function [p] = get.Position(this)
            p = this.SO3.Position;
        end
        function set.Position(this,p)
            this.SO3.Position = p;
        end
        function [q] = get.Rotation(this)
            q = this.SO3.Rotation;
        end
        function set.Rotation(this,q)
            this.SO3.Rotation = q;
        end
        function [s] = get.Scale(this)
            s = this.SO3.Scale;
        end
        function set.Scale(this,s)
            this.SO3.Scale = s;
        end
        
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
        % Position & rotation
        function [T] = GetWorldMatrix(this)
            % Get the world transformation matrix.

            % [To fix after parentage]
            T = this.GetLocalMatrix();
        end
        function [this] = SetWorldMatrix(this,m)
            % Set the World transformation matrix.

            % [To fix after parentage]
            this.SetLocalMatrix(m);
        end
        function [p] = GetWorldPosition(this)
            % This transform multiplied by all its parents

            % [To fix after parentage]
            p = this.GetLocalPosition();
        end
        function [this] = SetWorldPosition(this,p)
            
            % [To fix after parentage]
            this.SetLocalPosition(p);
        end
        function [r] = GetWorldRotation(this)
            % return the current world-frame rotation

            % [To fix after parentage]
            r = this.GetLocalRotation();
        end
        function [this] = SetWorldRotation(this,q)
            % Allow setting of the rotation in the world frame.

            % [To fix after parentage]
            this.SetLocalRotation(q);
        end
        function [R] = GetWorldRotationMatrix(this)
            % Get the local rotation from the transform.

            % [To fix after parentage]
            R = this.SO3.ToRotationMatrix();
        end
        function [this] = SetWorldRotationMatrix(this,R)
            % Get the local rotation from the transform.

            % [To fix after parentage]
            this.SO3.SetRotationMatrix(R);
        end
        % Scaling
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
        % Position & rotation
        function [T] = GetLocalMatrix(this)
            % Get the local transform matrix, this matrix already
            % contains the rotation,translation and scaling.
            T = this.SO3.ToMatrix();
        end
        function [this] = SetLocalMatrix(this,T)
            % Set the local transform matrix            
            this.SO3.SetMatrix(T);
        end
        function [p] = GetLocalPosition(this)
            % Get the local position from the transform.
            p = this.SO3.Position;
        end
        function [this] = SetLocalPosition(this,p)
            % Set the local position via the transform.
            this.SO3.Position = p;
        end
        function [p] = GetLocalRotation(this)
            % Get the local rotation from the transform.
            p = this.SO3.Rotation;
        end
        function [this] = SetLocalRotation(this,q)
            % Set the local rotation via the transform.
            this.SO3.Rotation = q;
        end
        function [R] = GetLocalRotationMatrix(this)
            % Get the local rotation from the transform.
            R = this.SO3.ToRotationMatrix();
        end
        function [this] = SetLocalRotationMatrix(this,R)
            % Get the local rotation from the transform.
            this.SO3.SetRotationMatrix(R);
        end
        % Scaling
        function [Ts] = GetLocalScaleMatrix(this)
            % Get the matrix representation of the scale
            Ts = this.SO3.ToScaleMatrix();
        end
        function [s] = GetLocalScale(this)
            % Get the local scale defined by this transform
            s = this.SO3.Scale;
        end
        function [this] = SetLocalScale(this,s)
            % Set the scale vector of the transform
            this.SO3.Scale = s;
        end
        % Visualisation
        function [h] = Plot(this,container)
            % Plot the transform
            if nargin < 2
                container = gca;
            end
            % Plot transform 
            h = this.SO3.Plot(container);
        end
    end
end

