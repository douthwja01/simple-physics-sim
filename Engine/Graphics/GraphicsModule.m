classdef (Abstract) GraphicsModule < Module
    % A base class for simple graphics APIs.

    properties
        Renderers;      
    end

    methods (Abstract)
        % Initialise the api
        [this] = Initialise(this,worldSize);
        % Update the representation
        [this] = Update(this);
    end

    methods
        % Get/set
        function set.Renderers(this,r)
            assert(isa(r,"Renderer"),"Expecting a valid renderer reference.");
            this.Renderers = r;
        end
        % Add or remove renderer components
        function [this] = AddRenderer(this,renderer)
            % Add a renderer to the graphics element.
            
            % Sanity check
            if isempty(renderer)
                return;
            end
            assert(isa(renderer,"Renderer"),"Expecting a valid 'Renderer' element.");
            % Add renderer to the set
            this.Renderers = vertcat(this.Renderers,renderer);
        end
        function [this] = RemoveRenderer(this,renderer)
            % Remove a rendere from the graphics world.
            
            % Sanity check
            if isempty(renderer)
                return;
            end
             % Sanity check
            assert(isa(renderer,"Renderer"),"Expecting a valid 'Renderer' element.");
            % Remove a given renderer.
            this.Renderers = this.Renderers(this.Renderers ~= renderer);
        end
    end
end