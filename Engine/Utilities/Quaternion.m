classdef Quaternion < handle
    %QUATERNION mathmatical primitive.
    %   Detailed explanation goes here
    
    properties (SetObservable = true)
        X = 1;
        Y = 0;
        Z = 0;
        W = 0;
    end
    properties (Dependent)
        IsSymbolic;
    end
    
    %% Main
    methods
        function [this] = Quaternion(x,y,z,w)
            %QUATERNION Construct an instance of a quaternion class
            %   This class is designed to provide a reusable rotation
            %   object for different geometric projects.

            % Input check
            if nargin > 2
                this.X = x;
                this.Y = y;
                this.Z = z;
                this.W = w;
            elseif nargin > 0
                assert(numel(x) == 4,"Expecting a vector [4x1].");
                this.X = x(1);
                this.Y = x(2);
                this.Z = x(3);
                this.W = x(4);
            end
        end
        % Get/sets
        function [flag] = get.IsSymbolic(this)
            flag = ...
                isa(this.X,"sym") || ...
                isa(this.Y,"sym") || ...
                isa(this.Z,"sym") || ...
                isa(this.W,"sym");
        end
        % Operators
        function [this] = mtimes(this,m)
            % Override the '*' operator

            if isscalar(m)
                this.X = this.X*m;
                this.Y = this.Y*m;
                this.Z = this.Z*m;
                this.W = this.W*m;
                return
            end
            if isa(m,"Quaternion")
                this = this.Multiply(m);
                return;
            end
            error("Quaternion is not currently multipliable by this type.");
        end
        function [this] = plus(this,p)

            if isscalar(p) && isnumeric(p)
                this.X = this.X + p;
                this.Y = this.Y + p;
                this.Z = this.Z + p;
                this.W = this.W + p;
                return;
            end
            if isa(p,"Quaternion")
                this.X = this.X + p.X;
                this.Y = this.Y + p.Y;
                this.Z = this.Z + p.Z;
                this.W = this.W + p.W;
                return;
            end

            error("Quaternion is not currently add-able to this type.");
        end
    end
    %% Instance methods
    methods
        % Utilities
        function [Q] = Multiply(this,v)
            % Calculate the product of two quaternions
            % Associated block:
            % "Quaternion Multiplication"
            % Multiply the quaternion elements

            assert(isa(v,"Quaternion"),"Expecting another quaternion.");

            q = this.GetVector();
            vData = v.ToVector();
            % Quaternion projection matrix
            qv = [vData(1), -vData(2), -vData(3), -vData(4);
                vData(2),  vData(1), -vData(4),  vData(3);
                vData(3),  vData(4),  vData(1), -vData(2);
                vData(4), -vData(3),  vData(2),  vData(1)]*q; 
            % Confirmed with matlab website
            
            % Create new quaternion
            Q = Quaternion(qv);
        end
        function [Qinv] = Inverse(this)
            % Calculate the quaternion inverse.
            q = this.GetVector();
            qSqr = norm(q);
            qInv = zeros(4,1);
            qInv(1) =  q(1)/qSqr;
            qInv(2) = -q(2)/qSqr;
            qInv(3) = -q(3)/qSqr;
            qInv(4) = -q(4)/qSqr;
            % Return a new quaternion reference
            Qinv = Quaternion(qInv);
        end
        function [Q] = Normalise(this)
            % This function normalises the quaternion
            q0 = this.GetVector();
            q0 = q0/sqrt(q0(1)^2 + q0(2)^2 + q0(3)^2 + q0(4)^2);
            Q = Quaternion(q0);
        end
        % Conversions
        function [R] = GetMatrix(this)
            % This function computes the rotation matrix of the quaternion
            % variables describing the 3D rotations of 3D body.

            % Output container
            R = zeros(3);
            if this.IsSymbolic()
                R = sym(R);
            end
            q = this.GetVector();
            % Define the quaternion rotation matrix
            R(1,1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;    
            R(1,2) = 2*(q(2)*q(3) - q(1)*q(4));
            R(1,3) = 2*(q(1)*q(3) + q(2)*q(4));
            R(2,1) = 2*(q(2)*q(3) + q(1)*q(4));
            R(2,2) = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
            R(2,3) = 2*(q(3)*q(4) - q(1)*q(2));
            R(3,1) = 2*(q(2)*q(4) - q(1)*q(3));
            R(3,2) = 2*(q(1)*q(2) + q(3)*q(4));
            R(3,3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;
        end
        function [phi,theta,psi] = GetEulersAngles(this)
            % For convenience 
            q = this.GetVector();
            % Compute the euler rotation from a unit quaternion
            phi = atan2(2*(q(1)*q(2) + q(3)*q(4)),(1 - 2*(q(2)^2 + q(3)^2)));
            theta = asin(2*(q(1)*q(3) - q(4)*q(2)));
            psi = atan2(2*(q(1)*q(4) + q(2)*q(3)),(1 - 2*(q(3)^2 + q(4)^2)));
        end
        function [q] = GetVector(this)
            % Put the components in an array
            q = [this.X;this.Y;this.Z;this.W];
        end
    end

    %% (Static) Support methods
    methods (Static)
        % Operations
        function [dQ] = Rate(Q,omega)
            % Compute the quaternion differential

            % Sanity check
            assert(isa(Q,"Quaternion"),"Expecting a valid quaternion.");
            assert(IsColumn(omega,3),"Expecting a body axis rate [3x1].");

            q0 = Q.GetVector();
            % Rewritten to allow multiplication by omega_b directly (of [4x3])
            Jq = 0.5*[-q0(2), -q0(3), -q0(4);
                q0(1), -q0(4),  q0(3);
                q0(4),  q0(1), -q0(2);
                -q0(3), -q0(2),  q0(1)];
            % The differential
            dQ = Quaternion(Jq*omega);
        end
        function [Q] = Zero()
            Q = Quaternion();
        end
        function [Q] = Random()
            % Generate a random quaternion
            e = 2*pi*RandZero(3);
            Q = Quaternion.FromEulers(e(1),e(2),e(3));
        end
        % Conversions
        function [Q] = FromRotationMatrix(R)
            % This function is designed to convert from a rotation matrix
            % to an equivalent quaternion. This function is also parallel
            % to "rotm2quat.m".

            % Sanity check
            %assert(IsRotationMatrix(R),"Expecting a valid rotation matrix [3x3].");

            if ~isa(R,"sym")
                R = NormaliseRotationMatrix(R);
            end

            % The trace
            tr = R(1,1) + R(2,2) + R(3,3);

            % Container
            q = zeros(4,1);
            if isa(R,"sym")
                q = sym(q);
                % Assume condition #1
                S = sqrt(tr + 1.0) * 2; 
                q(1) = 0.25 * S;
                q(2) = (R(3,2) - R(2,3)) / S;
                q(3) = (R(1,3) - R(3,1)) / S;
                q(4) = (R(2,1) - R(1,2)) / S;
                % Create the quaternion object
                Q = Quaternion(q(1),q(2),q(3),q(4));
                return
            end

            % OUTPUT Matches rotm2quat
            if tr > 0 
                S = sqrt(tr + 1.0) * 2; 
                q(1) = 0.25 * S;
                q(2) = (R(3,2) - R(2,3)) / S;
                q(3) = (R(1,3) - R(3,1)) / S;
                q(4) = (R(2,1) - R(1,2)) / S;
                % is valid
            elseif ((R(1,1) > R(2,2)) && (R(1,1) > R(3,3)))
                S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*q(2)
                q(1) = (R(3,2) - R(2,3)) / S;
                q(2) = 0.25 * S;
                q(3) = (R(1,2) + R(2,1)) / S;
                q(4) = (R(1,3) + R(3,1)) / S;
                % is valid
            elseif (R(2,2) > R(3,3))
                S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S=4*q(3)
                q(1) = (R(1,3) - R(3,1)) / S;
                q(2) = (R(1,2) + R(2,1)) / S;
                q(3) = 0.25 * S;
                q(4) = (R(2,3) + R(3,2)) / S;
                % to validate
            else
                S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S=4*q(4)
                q(1) = (R(2,1) - R(1,2)) / S;
                q(2) = (R(1,3) + R(3,1)) / S;
                q(3) = (R(2,3) + R(3,2)) / S;
                q(4) = 0.25 * S;
                % to validate
            end
            % Create the quaternion object
            Q = Quaternion(q);
        end
        function [Q] = FromEulers(phi,theta,psi)
            % Generate a quaternion of the equivalent euler angles.

            % Abbreviations for the various angular functions
            cy = cos(psi * 0.5);
            sy = sin(psi * 0.5);
            cp = cos(theta * 0.5);
            sp = sin(theta * 0.5);
            cr = cos(phi * 0.5);
            sr = sin(phi * 0.5);
            % Compute the components
            X = cr * cp * cy + sr * sp * sy;
            Y = sr * cp * cy - cr * sp * sy;
            Z = cr * sp * cy + sr * cp * sy;
            W = cr * cp * sy - sr * sp * cy;
            % Assign to quaternion
            Q = Quaternion(X,Y,Z,W);
        end
    end
end

