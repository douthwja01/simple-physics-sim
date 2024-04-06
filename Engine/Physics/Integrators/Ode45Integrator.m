
classdef Ode45Integrator < IntegratorModule
    properties (Constant)
        Name = "Wrapper for the inbuilt ode45 method."
    end
    properties
        options = odeset('RelTol',1e-5,'AbsTol',1e-6); %,"OutputFcn",@SimulatorPlotHandle);
    end
    methods
        function [X] = Integrate(this,t0,X0,U0)
            % This function completes a continuous simulation (using the
            % ode45 function) against the target model and given simulation
            % settings.

            % Solve the open-loop simulation
            [Y] = ode45(this.Function,...
                [t0 (t0+this.TimStep)],...
                X0,...
                this.options,...
                U0);
            % Return only the final state array
            X = Y(:,end);
        end
    end
end
