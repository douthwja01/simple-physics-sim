

classdef AxisCode < uint16

    methods (Static)
        function [v] = GetVector(code)
            % This function gets the numeric vector corresponding to the
            % axis code.

            % Sanity check
            assert(isa(code,"AxisCode"),"Expecting a valid axis-code.");

            switch code
                case AxisCode.X
                    v = [1;0;0];
                    return;
                case AxisCode.Y
                    v = [0;1;0];
                    return;
                case AxisCode.Z
                    v = [0;0;1];
                    return;
                otherwise
                    error("Not a valid axis-code.");
            end
        end
    end

    enumeration
        Unknown(0);
        X(1);
        Y(2);
        Z(3);
    end
end