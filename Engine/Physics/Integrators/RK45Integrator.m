classdef RK45Integrator < IntegratorModule
    properties (Constant)
        Name = "Runge-Kutta-Fehlberg method (RK45)";
    end
    properties
        Epsilon = 1E-5;
    end
    methods
        function [X] = Integrate(this,t0,X0,U0)
            % This function evaluates the provided function given the
            % current time, initial state and input.

            h = this.TimeStep;
            
            R = inf;
            while R > this.Epsilon
                h = min(h, 2-t0);
                k1 = h*this.Function(t0,X0,U0);
                k2 = h*this.Function(t0+h/4,X0+k1/4,U0);
                k3 = h*this.Function(t0+3*h/8,X0+3*k1/32+9*k2/32,U0);
                k4 = h*this.Function(t0+12*h/13, X0+1932*k1/2197-7200*k2/2197+7296*k3/2197,U0);
                k5 = h*this.Function(t0+h, X0+439*k1/216-8*k2+3680*k3/513-845*k4/4104,U0);
                k6 = h*this.Function(t0+h/2, X0-8*k1/27+2*k2-3544*k3/2565+1859*k4/4104-11*k5/40,U0);
                % Calculate the state changes
                Xa = X0 + 25*k1/216+1408*k3/2565+2197*k4/4104-k5/5;
                Xb = X0 + 16*k1/135+6656*k3/12825+28561*k4/56430-9*k5/50+2*k6/55;
                % Resolution parameters
                R = abs(Xa-Xb)/h;
                delta = 0.84*(this.Epsilon./R).^(1/4);
                % Modify the time-step
                h = delta*h;
            end
            % Increment to the next state
            X = Xa;
        end
    end
end


% function rk45
% epsilon = 0.00001;
% h = 0.2;
% t = 0;
% w = 0.5;
% i = 0;
% fprintf(’Step %d: t = %6.4f, w = %18.15f\n’, i, t, w);
% while t<2
% h = min(h, 2-t);
% k1 = h*f(t,w);
% 4
% k2 = h*f(t+h/4, w+k1/4);
% k3 = h*f(t+3*h/8, w+3*k1/32+9*k2/32);
% k4 = h*f(t+12*h/13, w+1932*k1/2197-7200*k2/2197+7296*k3/2197);
% k5 = h*f(t+h, w+439*k1/216-8*k2+3680*k3/513-845*k4/4104);
% k6 = h*f(t+h/2, w-8*k1/27+2*k2-3544*k3/2565+1859*k4/4104-11*k5/40);
% w1 = w + 25*k1/216+1408*k3/2565+2197*k4/4104-k5/5;
% w2 = w + 16*k1/135+6656*k3/12825+28561*k4/56430-9*k5/50+2*k6/55;
% R = abs(w1-w2)/h;
% delta = 0.84*(epsilon/R)ˆ(1/4);
% if R<=epsilon
% t = t+h;
% w = w1;
% i = i+1;
% fprintf(’Step %d: t = %6.4f, w = %18.15f\n’, i, t, w);
% h = delta*h;
% else
% h = delta*h;
% end
% end