
classdef Entity < matlab.mixin.Heterogeneous & handle
    % A primitive representing a simulation object.
    
    properties
        Name = "Unnamed";
        Uuid;
    end
    properties (SetAccess = private)
        Elements = Element.empty;
    end
    methods
        function [this] = Entity(position)
            % Create an entity object

            % Generate a random integer 
            this.Uuid = RandIntOfLength(6);
            
            % Constructor for entities.
            if nargin > 0
                tf = Transform(position,eye(3));
            else
                tf = Transform();
            end
            % Add the transform
            this.AddElement(tf);
        end
        % Element interaction
        function [element] = GetElement(this,className)
            assert(isstring(className),"Expecting a string class name");
            % Get and return a singular element defined by a class name.
            element = this.GetElements(className);
            if numel(element) > 1
                element = element(1);
            end
        end
        function [element] = GetElements(this,className)
            assert(isstring(className),"Expecting a string class name");
            % Get all elements that are a given 
            element = this.Elements(IsClass(this.Elements,className));
        end
        function [element] = AddElement(this,element)
            assert(isa(element,"Element"),"Expecting a valid entity 'Element'.");
            % Associate
            element.AddToEntity(this);
            this.Elements = vertcat(this.Elements,element);
        end
        function [this] = RemoveElement(this,element)
            assert(isa(element,"Element"),"Expecting a valid entity 'Element'.");
            % Disassociate
            element.RemoveFromEntity(this);
            this.Elements = this.Elements(this.Elements ~= element);
        end
    end
end