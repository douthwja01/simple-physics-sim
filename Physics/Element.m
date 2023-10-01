
classdef (Abstract) Element < matlab.mixin.Heterogeneous & handle
    % A component applicable to an entity.
    
    properties (SetAccess = private)
        Entity
    end
    
    methods
        function [this] = AddToEntity(this,entity)
            this.Entity = entity;
        end
        function [this] = RemoveFromEntity(this)
            this.Entity = [];
        end
    end
end