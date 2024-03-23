classdef PrismaticJoint < ActuatedJoint
    %PRISMATICJOINT Prismatic-Joint definition.
    %   This is a primitive joint class, characterised by a single 
    %   (translational) degree of freedom.
    
    properties (Constant)
        DegreesOfFreedom = 1;
        NumberOfInputs = 1;
    end
    
    methods
        function [this] = PrismaticJoint()
            %PRISMATICJOINT Construct an instance of this class
            
            % Call the parent class
            [this] = this@ActuatedJoint();
        end
    end
end

