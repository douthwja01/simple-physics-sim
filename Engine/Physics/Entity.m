
classdef Entity < matlab.mixin.Heterogeneous & handle
    % A primitive representing a simulation object.
    
    properties
        Name = "Unnamed";
        Uuid;
    end
    properties (SetAccess = private)
        Transform = Transform.empty;
        Elements = Element.empty;
    end
    methods
        function [this] = Entity(name)
            % Create an entity object

            % Assign name
            if nargin > 0
                assert(isstring(name),"Expecting a valid name string.");
                this.Name = name;
            end

            % Generate a random integer 
            this.Uuid = RandIntOfLength(6);
            % Add the transform (special-case)
            this.Transform = Transform(zeros(3,1),eye(3));
            % Also add it to the elements (so it can be found)
            this.AddElement(this.Transform);
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
            element.AssignEntity(this);
            this.Elements = vertcat(this.Elements,element);
        end
        function [this] = RemoveElement(this,element)
            assert(isa(element,"Element"),"Expecting a valid entity 'Element'.");
            % Disassociate
            element.UnassignEntity(this);
            this.Elements = this.Elements(this.Elements ~= element);
        end
    end
end