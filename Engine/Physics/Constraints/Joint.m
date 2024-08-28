classdef (Abstract) Joint < Element
    %JOINT Joint definition
    %   This is a base class for all joint constraint primitives.
    
    properties (Abstract,Constant)
        Code;
        DegreesOfFreedom;
    end   
    properties
        Location = SO3.empty; % The joint location in the entity frame.
    end
    
    methods
        function [this] = Joint()
            %JOINT Construct an instance of this class.

            % Assign the joint location
            this.Location = SO3.Identity();
        end
        % Get/sets
        function set.Location(this,l)
            assert(isa(l,"SO3"),"Expecting a valid pose for the joints position.");
            this.Location = l;
        end
        function [T] = GetJointTransformation(this)
            % This function computes the total joint transformation 
            T = this.Location.GetMatrix();                                  % Just return its location
        end
    end
    methods (Abstract)
        [S] = GetMotionSubspace(this);
        [T] = GetConstraintSubspace(this);
    end
end

