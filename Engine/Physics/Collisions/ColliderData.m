classdef ColliderData < event.EventData
    %ColliderData is a simple helper class the provides one persective of a
    %collider event to avoid confusion.
   
    properties
        Collider = Collider.empty();
    end

    methods
        function [this] = ColliderData(collider)
            % CONSTRUCTOR - Construct an instance of the collider data
            % class.
            
           this.Collider = collider;
        end
    end
end