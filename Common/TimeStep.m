classdef TimeStep
    %TIMESTEP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Time = 0.0;
        TimeDelta = 0.01;
        Step = 1;
    end
    
    methods
        function [this] = TimeStep(time,delta,step)
            if nargin > 0
                this.Time = time;
            end
            if nargin > 1
                this.TimeDelta = delta;
            end
            if nargin > 2
                this.Step = step;
            end
        end
        function [this] = IncrementBy(this,dt)
            this.TimeDelta = dt;
            this = this.Increment();
        end
        function [this] = Increment(this)
            this.Time = this.Time + this.TimeDelta;
            this.Step = this.Step + 1;
        end
    end
end

