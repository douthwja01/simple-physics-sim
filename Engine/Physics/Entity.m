
classdef Entity < matlab.mixin.Heterogeneous & handle
    % A primitive representing a simulation object.
    
    properties
        Name = "Unnamed";
        Uuid;
    end
    properties 
        Transformation = Element.empty;
        Renderer = Element.empty;
        RigidBody = Element.empty;
        Collider = Element.empty;
        Joints = [];
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
            % Create a transform
            this.Transformation = Transformation(this);
            this.Renderer = MeshRenderer();
        end
        % Get/sets
        function set.Uuid(this,u)
            assert(isinteger(u),"Expecting a valid identity code.");
            this.Uuid = u;
        end
        function set.Transformation(this,pose)
            assert(isa(pose,"Transformation"),"Expecting a valid transform element.");
            this.Transformation = pose;
            pose.AssignEntity(this);
        end
        function set.Renderer(this,r)
            assert(isa(r,"MeshRenderer"),"Expecting a valid renderer element.");
            this.Renderer = r;
            r.AssignEntity(this);
        end
        function set.RigidBody(this,r)
            assert(isa(r,"RigidBody"),"Expecting a valid rigidbody element.");
            this.RigidBody = r;
            r.AssignEntity(this);
        end
        function set.Collider(this,c)
            assert(isa(c,"Collider"),"Expecting a valid collider element.");
            this.Collider = c;
            c.AssignEntity(this);
        end
        function set.Joints(this,j)
            assert(isa(j,"Joint"),"Expecting a valid joint element.");
            this.Joints = j;
            for i = 1:numel(j)
                j(i).AssignEntity(this);
            end
        end
    end
end

% Entity-Component-System (ECS) logic
%         % Element interaction
%         function [element] = GetElement(this,className)
%             assert(isstring(className),"Expecting a string class name");
%             % Get and return a singular element defined by a class name.
%             element = this.GetElements(className);
%             if numel(element) > 1
%                 element = element(1);
%             end
%         end
%         function [element] = GetElements(this,className)
%             assert(isstring(className),"Expecting a string class name");
%             % Get all elements that are a given 
%             element = this.Renderer(IsClass(this.Renderer,className));
%         end
%         function [element] = AddElement(this,element)
%             assert(isa(element,"Element"),"Expecting a valid entity 'Element'.");
%             % Associate
%             element.AssignEntity(this);
%             this.Renderer = vertcat(this.Renderer,element);
%         end
%         function [this] = RemoveElement(this,element)
%             assert(isa(element,"Element"),"Expecting a valid entity 'Element'.");
%             % Disassociate
%             element.UnassignEntity(this);
%             this.Renderer = this.Renderer(this.Renderer ~= element);
%         end
%     end