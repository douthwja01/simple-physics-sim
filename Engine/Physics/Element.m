
classdef (Abstract) Element < matlab.mixin.Heterogeneous & handle
    % A component applicable to an entity.
    
    properties (SetAccess = private)
        Entity;     % The associated entity
    end
    properties (Dependent)
        Transform;       % The entity's transform
    end
    
    methods
        function [this] = Element(entity)
            if nargin > 0
                this.Entity = entity;
            end
        end
        function [p] = get.Transform(this)
            p = this.Entity.Transform;
        end
        function [this] = AssignEntity(this,entity)
            assert(isa(entity,"Entity"),"Expecting a valid entity.");
            this.Entity = entity;
        end
        function [this] = UnassignEntity(this)
            this.Entity = Element.empty;
        end
    end
end