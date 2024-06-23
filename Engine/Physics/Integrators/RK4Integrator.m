classdef RK4Integrator < Integrator
    % RK4INTEGRATOR - An integrator class that uses the RK4 method.
    properties (Constant)
        Name = "Runge-Kutta 4th-Order Algorithm";
    end
    methods (Static, Access = protected)
        function [objectData] = IntegrateObject(objectData,dt)
            % Update the object using the verlet method.

            error("Not implemented.");
        end
    end
end
%     methods (Access = protected)
%         function [this] = IntegrateTransform(this,transformation,dt)
%             % This function computes the RK4 method against a given
%             % transform to integrate its state-change over-time.
% 
%             % Get the initial frame-state
%             p0 = transformation.GetWorldPosition();
%             v0 = transformation.Velocity;
%             a0 = transformation.Acceleration;
% 
%             % Wrapper
%             X0 = [p0;v0;a0];
%             % Integrate
%             dX = this.IntegrateFun(@RK4Integrator.MotionStep,dt,0,X0);
%             % [SOMETHING IS WRONG HERE
%             X = X0 + dX*dt;
%             % Reapply
%             transformation.SetWorldPosition(X(1:3,1));
%             transformation.Velocity = X(4:6,1);
%             transformation.Acceleration = X(7:9,1);        
%         end
%     end
%     methods (Static)
%         function [X] = IntegrateFunWithInputs(fun,dt,t0,X0,U0)
%             % This function provide the raw method, with inputs
%             
%             % Calculate the RK4 gains
%             k1 = fun(t0,X0,U0);
%             k2 = fun(t0+dt/2,X0+k1*dt/2,U0);
%             k3 = fun(t0+dt/2,X0+k2*dt/2,U0);
%             k4 = fun(t0+dt,X0+k3*dt,U0);
%             % The next state
%             X = X0 +(k1/6+k2/3+k3/3+k4/6)*dt;
%         end
%         function [X] = IntegrateFun(fun,dt,t0,X0)
%             % This function provides the raw method.
%             
%             % Calculate the RK4 gains
%             k1 = fun(t0,X0);
%             k2 = fun(t0+dt/2,X0+k1*dt/2);
%             k3 = fun(t0+dt/2,X0+k2*dt/2);
%             k4 = fun(t0+dt,X0+k3*dt);
%             % The next state
%             X = X0 +(k1/6+k2/3+k3/3+k4/6)*dt;
%         end
%         function [dX] = MotionStep(dt,X0)
%             % Simple motion differentials
% 
%             % X - [p;v;a]
%             v_0 = X0(4:6,1);
%             a_0 = X0(7:9,1);
%             
%             da = zeros(3,1);
%             dv = a_0*dt;
%             dp = v_0 + dv;
%             % Collect the differentials
%             dX = [dp;dv;da];
%         end
%     end
% end