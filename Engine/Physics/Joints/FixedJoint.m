classdef FixedJoint < Joint
    %FIXEDJOINT Fixed-Joint definition.
    %   This is a primitive joint class, characterised by its unique
    %   zero degree of freedom configuration.
        
    properties (Constant)
        DegreesOfFreedom = 0;
        Code = JointCode.Fixed;
    end

    methods
        function [this] = FixedJoint()
            %FIXEDJOINT Construct an instance of this class
            
            % Call the parent class
            [this] = this@Joint();
        end
    end
end

