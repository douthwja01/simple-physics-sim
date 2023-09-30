
classdef (Abstract)SimObject < handle
    properties (SetAccess = protected)
        Transform;
        Handle = gobjects(1);
    end

    methods (Abstract)
        [this] = Update(this,dt);
    end

    methods
        function [this] = SimObject(varargin)
            % Object constructor

            % Populate the transform
            this.Transform = Transform();
        end
    end
end