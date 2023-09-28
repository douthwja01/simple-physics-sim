
classdef SimLink < matlab.mixin.Heterogeneous & handle

    properties
        Distance = 1.1;
        A = SimObject.empty;
        B = SimObject.empty;
    end
    methods
        function [this] = SimLink(objectA,objectB)

            assert(isa(objectA,"SimObject"),"Expecting a valid sim-object.");
            assert(isa(objectB,"SimObject"),"Expecting a valid second sim-object.");
            this.A = objectA;
            this.B = objectB;
        end
        function [this] = Apply(this)

            axis = this.B.Position - this.A.Position;
            distance = norm(axis);
            unitAxis = axis/distance;
            % Modify the positions
            this.A.position = this.A.position + 0.5*delta*unitAxis;
            this.B.position = this.B.position - 0.5*delta*unitAxis;
        end
    end
end