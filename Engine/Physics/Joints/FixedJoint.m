classdef FixedJoint < Joint
    %FIXEDJOINT Fixed-Joint definition.
    %   This is a primitive joint class, characterised by its unique
    %   zero degree of freedom configuration.
        
    methods
        function [this] = FixedJoint()
            %FIXEDJOINT Construct an instance of this class
            
            % Call the parent class
            [this] = this@Joint();
        end
    end
end

