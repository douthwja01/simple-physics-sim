
classdef (Abstract) Element < matlab.mixin.Heterogeneous & handle
    % A component applicable to an entity.
    
    properties (SetAccess = private)
        Entity
    end
    
    methods
        function [this] = AssignEntity(this,entity)
            assert(isa(entity,"Entity"),"Expecting a valid entity.");
            this.Entity = entity;
        end
        function [this] = UnassignEntity(this)
            this.Entity = Entity.empty;
        end
    end
end