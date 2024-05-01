
classdef (Abstract) Element < matlab.mixin.Heterogeneous & handle
    % A component applicable to an entity.
    
    properties (SetAccess = private)
        Entity;             % The associated entity
    end
    properties (Dependent)
        Transform;          % The entity's transform
    end
    properties (Hidden)
        Uid;                % Unique reference (must be generated)
    end
    % Main
    methods
        function [this] = Element(entity)
            % CONSTRUCTOR - Create an instance of the element class.
            if nargin > 0
                this.Entity = entity;
            end
        end
        % Get/sets
        function [p] = get.Transform(this)
            p = this.Entity.Transform;
        end
        function [uid]  = get.Uid(this)
            if isempty(this.Uid)
                this.Uid = this.CreateRandomUID(5);
            end
            uid = this.Uid;
        end
        function [this] = AssignEntity(this,entity)
            assert(isa(entity,"Entity"),"Expecting a valid entity.");
            this.Entity = entity;
        end
        function [this] = UnassignEntity(this)
            this.Entity = Element.empty;
        end
    end
    % Utilities
    methods
        function [flag] = IsSame(this,eParam)                        	    
            % This function compares the data given to fields of this
            % element.
            flag = false;
            % Catch bad inputs
            if isempty(this) || isempty(eParam)
                return
            end
            if isa(eParam,"Element")
                flag = this.Uid == eParam.Uid;
                return
            end
            if isa(eParam,"uint64")
                flag = this.Uid == eParam;
                return
            end
            if ~isprop(this,eParam)
                error("Comparative parameter not recognised.");
            end
        end
    end
    % Internals
    methods (Static, Access = private)
        function [uid]  = CreateRandomUID(n)
            % Create a random unique reference
            uid = uint64(randi([0,1E15],1,1));
            t = char(string(uid));
            assert(n <= length(t),"Expecting uid length in range 0-15");
            if nargin < 1
                n = length(t);
            end
            uid = double(string(t(1:n)));
            uid = uint64(uid);
        end
    end
end