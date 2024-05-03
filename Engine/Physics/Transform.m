classdef Transform < TreeElement
    %TRANSFORM The geometric representation of the entity in 3D space.
    
    % Kinematic properties (simple containers)    
    properties
        % Kinematics
        Velocity = zeros(3,1);
        AngularVelocity = zeros(3,1);
        Acceleration = zeros(3,1);
        AngularAcceleration = zeros(3,1);
        IsStatic = false;
    end

    properties (Hidden)
%         PriorPosition = [];
        Local = SO3.empty;
        Inertial = SO3.empty;
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
            this.Local = SO3.Zero;
            this.Inertial = SO3.Zero;
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
        % Gets
%         function [T]    = GetWorldMatrix(this)
%             % Get the world transformation matrix.
% 
%             % Get the local matrix
%             T = this.GetLocalMatrix();
%             if this.NumberOfParents == 0
%                 return
%             end
%             % Get the world transformation of the parent
%             Tp = this.Parent.GetWorldMatrix();
%             % Multiply by the parent matrix
%             T = Tp*T;
%         end
%         function [p]    = GetWorldPosition(this)
%             % This transform multiplied by all its parents
% 
%             % Get the local matrix
%             T = this.GetWorldMatrix();
%             if this.NumberOfParents == 0
%                 p = this.Position;
%                 return
%             end
%             % Extract from the transform
%             p = T(1:3,4);
%         end    
%         function [q]    = GetWorldRotation(this)
%             % return the current world-frame rotation
% 
%             ql = this.GetLocalRotation();
%             if this.NumberOfParents == 0
%                 q = ql;
%                 return
%             end
%             % Get the parent rotation
%             qParent = this.Parent.GetWorldRotation();
%             % Multiple local with parent world
%             q = qParent.Multiply(ql);
% 
%         end
%         function [Ts]   = GetWorldScaleMatrix(this)
%             % Get the matrix representation of the scale
% 
%             Tl = this.GetLocalScaleMatrix();
%             if this.NumberOfParents == 0
%                 Ts = Tl;
%                 return
%             end
%             Tp = this.Parent.GetWorldScaleMatrix();
%             % Multiply by parent scale matrix
%             Ts = Tp*Tl;
%         end
%         function [s]    = GetWorldScale(this)
%             % Get the scale in the world-frame
% 
%             sl = this.GetLocalScale();
%             if this.NumberOfParents == 0
%                 s = sl;
%                 return
%             end
% 
%             sp = this.Parent.GetWorldScale();
%             % Multiply by parent scale matrix
%             s = sp.*sl;
%         end
%         % Sets
%         function [this] = SetWorldMatrix(this,Tw)
%             % Set the World transformation matrix.
% 
%             % Get the local matrix
%             if this.NumberOfParents == 0
%                 this.SetLocalMatrix(Tw);
%                 return
%             end
%             % Get the current world matrix
%             Tw_inv = this.Parent.GetWorldMatrix;
%             % Get the difference between them [To Check]
%             Tl = Tw/Tw_inv;
%             % [To fix after parentage]
%             this.SetLocalMatrix(Tl);
%         end
%         function [this] = SetWorldPosition(this,p)
%             % Allow the setting of the transform's world position.
% 
%             %p_parent = this.Parent.GetWorldPosition();
%             
%             pLocal = this.InverseTransformPoint(p);
% 
%             % [To fix after parentage]
%             this.SetLocalPosition(pLocal);
%         end
%         function [this] = SetWorldRotation(this,q)
%             % Allow setting of the rotation in the world frame.
% 
%             % Get the local matrix
%             if this.NumberOfParents == 0
%                 this.SetLocalRotation(q);
%                 return
%             end
%             % Get the parent rotation
%             qParent = this.Parent.GetWorldRotation();
%             % Subtract the parent rotation
%             qLocal = q.Multiply(qParent);
%             % Assign the local rotation
%             this.SetLocalRotation(qLocal);
%         end
%         function [this] = SetWorldScale(this,s)
%             % Allow the setting of the world scale.
% 
%             % Parent check
%             if this.NumberOfParents == 0
%                 this.SetLocalScale(s);
%                 return
%             end
%             % Get the parent world scale
%             sP = this.Parent.GetWorldScale();
%             % Extract the implicit local scales
%             sl = s./sP;
%             % Assign the local scale
%             this.SetLocalScale(sl);
%         end
    end
    % (Local) Representation
    methods
        % Gets
%         function [T] = GetLocalMatrix(this)
%             % Get the local transform matrix, this matrix already
%             % contains the rotation,translation and scaling.
%             T = this.Local.ToMatrix();
%         end
%         function [p] = GetLocalPosition(this)
%             % Get the local position from the transform.
%             p = this.Local.Position;
%         end      
%         function [p] = GetLocalRotation(this)
%             % Get the local rotation from the transform.
%             p = this.Local.Rotation;
%         end    
%         function [Ts] = GetLocalScaleMatrix(this)
%             % Get the matrix representation of the scale
%             Ts = this.Local.ToScaleMatrix();
%         end  
%         function [s] = GetLocalScale(this)
%             % Get the local scale defined by this transform
%             s = this.Local.Scale;
%         end      
%         % Sets
%         function [this] = SetLocalMatrix(this,T)
%             % Set the local transform matrix            
%             this.Local = SO3.FromMatrix(T);
%             this.HasChanged = true;
%         end
%         function [this] = SetLocalPosition(this,p)
%             % Set the local position via the transform.
%             this.Local.Position = p;
%             this.HasChanged = true;
%         end
%         function [this] = SetLocalRotation(this,q)
%             % Set the local rotation via the transform.
%             this.Local.Rotation = q;
%             this.HasChanged = true;
%         end
%         function [this] = SetLocalScale(this,s)
%             % Set the scale vector of the transform
%             this.Local.Scale = s;
%             this.HasChanged = true;
%         end
        % Visualisation
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

