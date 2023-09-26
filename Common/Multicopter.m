
classdef Multicopter

    methods (Static)
        function [pxn,angles]   = UniformNacellePositions(number,config,radius,height)
            % INPUTS:
            % number    - Number of nacelles
            % alignment - The configuration dictating the first position first
            % height    - The vertical offset of the nacelles "vertical arm height"
            % radius    - The horezontal offset of the nacelles "Horizontal arm distance"
            % OUTPUTS:
            % pxn       - Nacelle positions [3xn]
            % angles    - The corresponding planar rotations [1xn]

            % Sanity check
            assert(isnumeric(number) && number > 0,"The nacelle number is invalid.");
            % Get the angle vector
            [angles] = Multicopter.CreateUniformNacelleAngles(number,config);
            % Calculate the implied elevation angle
            planarElevationAngle = atan2(height,radius);
            % Calculate the implied body axis positions of the nacelles
            for i = 1:number
                pxn(:,i) = R_z(angles(i))*R_y(planarElevationAngle)*[1;0;0]*radius;
            end
        end
        function [angles]       = CreateUniformNacelleAngles(number,config)
            % Input sanity check
            assert(isnumeric(number) && number > 0,"The number of nacelles must be numeric.");
            % Get the angle vector
            %stepAngle = (2/number)*sym("pi","real");
            stepAngle = (2/number)*pi;
            switch config
                case AlignmentType.Coaxial
                    angles = linspace(0,(2*pi - stepAngle),number);
                case AlignmentType.Xflyer
                    angles = linspace((stepAngle)/2,(stepAngle*number - (stepAngle/2)),number);
                otherwise
                    error("Configuration not recognised.");
            end
        end
        function [directions]   = AlternateRotorDirections(n)
            % Get the rotation direction vector
            % (Assigning conventions here)
            [directions] = AlternateSeries(n);
            directions([directions == 0]) = -1;
        end
    end
end