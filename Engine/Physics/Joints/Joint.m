classdef (Abstract) Joint < Element
    %JOINT Joint definition
    %   This is a base class for all joint constraint primitives.
    
    properties (Abstract,Constant)
        Code;
        DegreesOfFreedom;
    end   
    properties
        Location = SO3.Zero; % The joint location in the entity frame.
    end
    
    methods
        function [this] = Joint()
            %JOINT Construct an instance of this class.

        end
        % Get/sets
        function set.Location(this,l)
            assert(isa(l,"SO3"),"Expecting a valid pose for the joints position.");
            this.Location = l;
        end
    end
end

