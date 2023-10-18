classdef (Abstract) Integrator < handle

    % Provide a means to integrate the system state
    methods (Abstract)
        [this] = Integrate(this,bodies,dt);
    end
end