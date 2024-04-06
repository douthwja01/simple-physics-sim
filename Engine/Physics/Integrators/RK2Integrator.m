
classdef RK2Integrator < IntegratorModule
    properties (Constant)
        Name = "Second-order Runge-Kutta Method (RK2)";
    end
    methods
        function [X] = Integrate(this,t0,X0,U0)
            % This function evaluates the provided function given the
            % current time, initial state and input.
            
            h = this.TimeStep;
            % Calculate the gains(stages)
            k1 = this.Function(t0,X0,U0);
            k2 = this.Function(t0+h,X0+h*k1,U0);
            % Calculate the next state
            X = X0 + (h/2)*(k1+k2);
        end
    end
end