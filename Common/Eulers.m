classdef Eulers < handle
    % Euler properties
    properties (SetObservable = true, AbortSet)
        eulers = zeros(3,1);
    end
    properties (Dependent)
        rotation;
    end
    %% Tools
    methods (Static)
        % Rotation matrix -> Eulers
        function [eulers] = FromRotation(R)
            % Rotation of the fixed to the global pose

            % Prepare output containers
%             eulers = zeros(3,1);
%             isSym = isa(R,"sym");
%             if isSym
%                 eulers = sym(eulers);
%             end

            % Sanity check
            assert(IsRotationMatrix(R),"Expecting a valid rotation matrix.");
 
            sy = sqrt(R(1,1) * R(1,1) +  R(2,1) * R(2,1));
            singular = sy < 1e-6;
            if ~singular || isSym
                x = atan2(R(2,1), R(1,1));
                y = atan2(-R(3,1), sy);
                z = atan2(R(3,2) , R(3,3));                
            else
                x = 0;
                y = atan2(-R(3,1), sy);
                z = atan2(-R(2,3), R(2,2));
            end
            % Collate
            eulers = [x; y; z];
        end
        % Eulers -> rotation matrix
        function [Reta] = ToRotation(phi,theta,psi)
            % INPUTS
            % phi   - Roll angle
            % theta - Pitch angle
            % psi   - Yaw angle
            % OUTPUT
            % Reta - Combined rotations about the xyz axes

            % Input sanity check
            if nargin == 3
                eta = [phi;theta;psi];
            elseif nargin == 1
                eta = phi; % phi = [phi,theta,psi]
            elseif nargin == 0
                eta = [sym('phi_t','real');sym('theta_t','real');sym('psi_t','real')];
            else
                error('Please provide either a vector of Euler angle or a series of Euler angles.');
            end

            % Check the inputs
            assert(numel(eta) == 3,'Euler angles must be provided by a vector [3x1].');
            assert(isa(eta,'sym') || isnumeric(eta),'Euler angles must be a symbolic or numeric value.');

            % Get the progressive rotation matrices
            Rx = R_x(eta(1));
            Ry = R_y(eta(2));
            Rz = R_z(eta(3));
            % Compute the rotations in Z-Y-X order
            Reta = Rz*Ry*Rx;
        end
    end
end