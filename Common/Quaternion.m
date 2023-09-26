classdef Quaternion < handle
    % Quaternion properties
    properties (SetObservable = true, AbortSet)                          	% Updating will trigger call back butonly on value change
        value = [1;0;0;0];
    end
    
    %% Main
    methods
        % Constructor
        function [this] = Quaternion(q)
           
            % Construct the transform
            if nargin > 0
                this.value = q;
            end
            % Add listener to report changes
            addlistener(this,"value","PostSet",@(src,evnt)OnValueChanged(this));
        end
        % Quaternion
        function set.value(this,q)
            assert(IsColumn(q,4),"Expecting a [4x1] quaternion vector.");
            this.value = q;
        end
        % Rotation
        function [R] = GetRotationMatrix(this)
            R = Quaternion.ToRotation(this.value);
        end
        function [this] = SetRotationMatrix(this,R)
            assert(IsSquare(R,3),"Expecting a [3x3] rotation matrix.");
            this.value = Quaternion.FromRotation(R);
        end
        function [this] = Reset(this)
            % Reset to default transform
            this.value = [1;0;0;0];
        end
    end

    methods (Access = private)
        function [this] = OnValueChanged(this)
            % This event is notified when the transform value is updated.
            notify(this,"ValueChanged");
        end
    end
    events (NotifyAccess = private)
        ValueChanged;
    end

    %% Tools
    methods (Static)
        % Quaternion -> Rotation matrix
        function [R] = ToRotation(q)
            % This function computes the rotation matrix of the quaternion
            % variables describing the 3D rotations of 3D body.

            % Input sanity check
            if nargin < 1
                q = sym('q%d',[4,1],'real');
            end
            assert(numel(q) == 4,'The quaternion must be of format [4x1]');
            
            isSym = false;
            if isa(q,"sym")
                isSym = true;
            else
                q = Quaternion.Unit(q);  % Normalise the quaternion
            end

            % Output container
            R = zeros(3);
            if isSym
                R = sym(R);
            end

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

            % Reduce where possible
%             R = SymTools.Reduce(R);
        end
        % Rotation matrix -> Quaternion
        function [q] = FromRotation(R)
            % This function is designed to convert from a rotation matrix
            % to an equivalent quaternion. This function is also parallel
            % to "rotm2quat.m".

            % Sanity check
            assert(IsRotationMatrix(R),"Expecting a valid rotation matrix [3x3].");

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
                % Reduce if possible
                q = SymTools.Reduce(q);
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
        end
        % Quaternion -> Eulers
        function [angles] = ToEulerAngles(q)
            % Sanity check
            assert(numel(q) == 4,"Expecting quaternion vector [4x1].");

            % Compute the euler rotation from a unit quaternion
            angles = zeros(3,1);
            angles(1) = atan2(2*(q(1)*q(2) + q(3)*q(4)),(1 - 2*(q(2)^2 + q(3)^2)));
            angles(2) = asin(2*(q(1)*q(3) - q(4)*q(2)));
            angles(3) = atan2(2*(q(1)*q(4) + q(2)*q(3)),(1 - 2*(q(3)^2 + q(4)^2)));
        end
        % Eulers -> quaternion
        function [q] = FromEulerAngles(xa,ya,za)
                        
            % Sanity check
            if nargin < 3
                assert(length(xa) == 3,"Expecting euler vector [3x1].");
                eulers = xa;
            else
                eulers = [xa,ya,za];
            end

            % Abbreviations for the various angular functions
            cy = cos(eulers(3) * 0.5);
            sy = sin(eulers(3) * 0.5);
            cp = cos(eulers(2) * 0.5);
            sp = sin(eulers(2) * 0.5);
            cr = cos(eulers(1) * 0.5);
            sr = sin(eulers(1) * 0.5);

            q = zeros(4,1);
            q(1) = cr * cp * cy + sr * sp * sy;
            q(2) = sr * cp * cy - cr * sp * sy;
            q(3) = cr * sp * cy + sr * cp * sy;
            q(4) = cr * cp * sy - sr * sp * cy;
        end
    end
    methods (Static)
        % Multiplication
        function [qv] = Multiply(q,v)
            % Calculate the product of two quaternions
            % Associated block:
            % "Quaternion Multiplication"
            % Multiply the quaternion elements

            assert(size(q,1) == 4 && size(v,1) == 4,...
                'Both quaternion must be provided as 4x1 column vectors')
            % Quaternion projection matrix
            qv = [v(1), -v(2), -v(3), -v(4);
                v(2),  v(1), -v(4),  v(3);
                v(3),  v(4),  v(1), -v(2);
                v(4), -v(3),  v(2),  v(1)]*q; % Confirmed with matlab website
        end
        % Unit
        function [q_hat] = Unit(q)
            % This function normalises the quaternion
            q_hat = q/sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
        end
        % Random
        function [q] = Random()
            b = 2*pi; a = -2*pi;
            r = (b-a).*rand(3,1) + a;
            q = Quaternion.FromEulerAngles(r);
            q = Quaternion.Unit(q);
        end
    end
end