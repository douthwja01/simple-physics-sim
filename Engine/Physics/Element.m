
classdef (Abstract) Element < matlab.mixin.Heterogeneous & handle
    % A component applicable to an entity.
    
    properties (SetAccess = private)
        Entity;     % The associated entity
        Transform;  % The entity's transform
    end
    
    methods
        function [this] = Element(entity)
            if nargin > 1
                this.Entity = entity;
            end
        end
        function [this] = AssignEntity(this,entity)
            assert(isa(entity,"Entity"),"Expecting a valid entity.");
            this.Entity = entity;
            this.Transform = entity.Transform;
        end
        function [this] = UnassignEntity(this)
            this.Entity = Entity.empty;
            this.Transform = Transform.empty;
        end
    end
end