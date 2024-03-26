classdef (Abstract) Joint < Element
    %JOINT Joint definition
    %   This is a base class for all joint constraint primitives.
    
    properties
        Location = Pose.empty; % The joint location in the entity frame.
    end
    
    methods
        function [this] = Joint()
            %JOINT Construct an instance of this class
        end
        % Get/sets
        function set.Location(this,l)
            assert(isa(l,"Pose"),"Expecting a valid pose for the joints position.");
            this.Location = l;
        end
    end
end

