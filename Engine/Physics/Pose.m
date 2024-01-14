classdef Pose < TreeElement
    %POSE Summary of this class goes here
    %   Detailed explanation goes here
    
    % Kinematic properties (simple containers)
    properties
        % Statics
        Transform = Element.empty;
        % Kinematics
        Velocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
        Acceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
        IsStatic = false;
    end
    properties (Hidden)
        PriorPosition = [];
    end

    methods
        function [this] = Pose(entity)
            %POSE Construct an instance of the pose-element
            % Detailed explanation goes here

            % Input check
            if nargin < 1
                entity = Entity.empty;
            end

            % Assign the entity
            [this] = this@TreeElement(entity);

            % Create a new transform object
            this.Transform = Transform();
        end
        % Get/sets
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
    methods
        % World representation
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
        % Local representation
        function [T] = GetLocalMatrix(this)
            % Get the local transform matrix
            T = this.Transform.GetMatrix();
        end
        function [this] = SetLocalMatrix(this,T)
            % Set the local transform matrix            
            this.Transform.SetMatrix(T);
        end
        function [p] = GetLocalPosition(this)
            % Get the local position from the transform.
            p = this.Transform.Position;
        end
        function [this] = SetLocalPosition(this,p)
            % Set the local position via the transform.
            this.Transform.Position = p;
        end
        function [p] = GetLocalRotation(this)
            % Get the local rotation from the transform.
            p = this.Transform.Quaternion;
        end
        function [this] = SetLocalRotation(this,q)
            % Set the local rotation via the transform.
            this.Transform.Quaternion = q;
        end
        function [s] = GetLocalScale(this)
            % Get the local scale defined by this transform
            s = this.Transform.Scale;
        end
        function [this] = SetLocalScale(this,s)
            % Set the scale vector of the transform
            this.Transform.Scale = s;
        end
        % Visualisation
        function [h] = Plot(this,container)
            % Plot the transform
            if nargin < 2
                container = gca;
            end
            % Plot transform 
            h = this.Transform.Plot(container);
        end
    end
end

