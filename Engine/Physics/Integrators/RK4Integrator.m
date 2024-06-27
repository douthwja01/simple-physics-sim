classdef RK4Integrator < Integrator
    % RK4INTEGRATOR - An integrator class that uses the RK4 method.

    % https://github.com/ange-yaghi/simple-2d-constraint-solver/blob/master/src/rk4_ode_solver.cpp

    properties (Constant)
        Name = "Runge-Kutta 4th-Order Algorithm";
    end
    properties (Access = private)
        InitialState;
        Stage = 1;
        Accumulator;
    end
    methods
        function [this] = RK4Integrator()
            % CONSTRUCTOR - Instantiate an instance of the integrator
            % class.

            % Create the parent.
            [this] = this@Integrator();
        end
        % Start the solver
        function [this] = Start(this,initialState,t_delta)
            % This function records the frame-constants ahead of the solve
            % method call and initialises the integration stages.

            this.InitialState = initialState;
            this.TimeDelta = t_delta;
            this.Stage = 1;
            this.Accumulator = initialState;
        end
        % Step the solver
        function [flag] = Step(this,state)
            % Set the dt of the system state to this solution step.

            switch (this.Stage)
                case 1
                    state.dt = 0.0;
                case 2
                    % Do nothing
                case 3
                    initial = this.InitialState;
                    t_delta = this.TimeDelta / 2;

                    for i = 1:state.NumberOfBodies
                        % --- 1st -> 0th ---
                        % Positions (linear)
                        state.p_x(i) = initial.p_x(i) + state.v_x(i)*t_delta;
                        state.p_y(i) = initial.p_y(i) + state.v_y(i)*t_delta;
                        state.p_z(i) = initial.p_z(i) + state.v_z(i)*t_delta;
                        % Calculate the quaternion rate
                        q0 = [initial.q_x(i);initial.q_y(i);initial.q_z(i);initial.q_w(i)];
                        w  = [state.w_x(i);state.w_y(i);state.w_z(i)];
                        dq = Integrator.QuaternionRateArray(q0,w);
                        % Positions (angular)
                        state.q_x(i) = initial.q_x(i) + dq(1)*t_delta;
                        state.q_y(i) = initial.q_y(i) + dq(2)*t_delta;
                        state.q_z(i) = initial.q_z(i) + dq(3)*t_delta;
                        state.q_w(i) = initial.q_w(i) + dq(4)*t_delta;

                        % --- 2nd -> 1st ---
                        % Velocities (linear)
                        state.v_x(i) = initial.v_x(i) + state.a_x(i)*t_delta;
                        state.v_y(i) = initial.v_y(i) + state.a_y(i)*t_delta;
                        state.v_z(i) = initial.v_z(i) + state.a_z(i)*t_delta;
                        % Velocities (angular)
                        state.w_x(i) = initial.w_x(i) + state.dw_x(i)*t_delta;
                        state.w_y(i) = initial.w_y(i) + state.dw_y(i)*t_delta;
                        state.w_z(i) = initial.w_z(i) + state.dw_z(i)*t_delta;
                    end
                    % Time-step update
                    state.dt = t_delta;
                case 4
                    initial = this.InitialState;
                    % Time-step is the Original time-delta
                    dt = this.TimeDelta;
                    for i = 1:state.NumberOfBodies
                        % --- 1st -> 0th ---
                        % Positions (linear)
                        state.p_x(i) = initial.p_x(i) + state.v_x(i)*dt;
                        state.p_y(i) = initial.p_y(i) + state.v_y(i)*dt;
                        state.p_z(i) = initial.p_z(i) + state.v_z(i)*dt;
                        % Calculate the quaternion rate
                        q0 = [initial.q_x(i);initial.q_y(i);initial.q_z(i);initial.q_w(i)];
                        w  = [state.w_x(i);state.w_y(i);state.w_z(i)];
                        dq = Integrator.QuaternionRateArray(q0,w);
                        % Positions (angular)
                        state.q_x(i) = initial.q_x(i) + dq(1)*dt;
                        state.q_y(i) = initial.q_y(i) + dq(2)*dt;
                        state.q_z(i) = initial.q_z(i) + dq(3)*dt;
                        state.q_w(i) = initial.q_w(i) + dq(4)*dt;

                        % --- 2nd -> 1st ---
                        % Velocities (linear)
                        state.v_x(i) = initial.v_x(i) + state.a_x(i)*dt;
                        state.v_y(i) = initial.v_y(i) + state.a_y(i)*dt;
                        state.v_z(i) = initial.v_z(i) + state.a_z(i)*dt;
                        % Velocities (angular)
                        state.w_x(i) = initial.w_x(i) + state.dw_x(i)*dt;
                        state.w_y(i) = initial.w_y(i) + state.dw_y(i)*dt;
                        state.w_z(i) = initial.w_z(i) + state.dw_z(i)*dt;
                    end
                    % Time-step update
                    state.dt = dt;
                otherwise
                    % Either complete or bad.
            end

            this.Stage = this.Stage + 1;

            flag = this.Stage == 4;
        end
        % Solve the current state
        function [state] = Solve(this,state)
            % This function solves the zeroth and 1st states from the
            % current 1st and 2nd order states.

            switch this.Stage
                case 1
                    stageWeight = 1;
                case 2
                    stageWeight = 2;
                case 3
                    stageWeight = 2;
                case 4
                    stageWeight = 1;
                otherwise
                    stageWeight = 0;
            end

            acc = this.Accumulator;

            Ct = this.TimeDelta * stageWeight / 6;

            for i = 1:state.NumberOfBodies
                % --- 1st -> 0th ---
                % Positions (linear)
                acc.p_x(i) = acc.p_x(i) + state.v_x(i)*Ct;
                acc.p_y(i) = acc.p_y(i) + state.v_y(i)*Ct;
                acc.p_z(i) = acc.p_z(i) + state.v_z(i)*Ct;
                % Calculate the quaternion rate
                q0 = [acc.q_x(i);acc.q_y(i);acc.q_z(i);acc.q_w(i)];
                w  = [state.w_x(i);state.w_y(i);state.w_z(i)];
                dq = Integrator.QuaternionRateArray(q0,w);
                % Positions (angular)
                acc.q_x(i) = acc.q_x(i) + dq(1)*Ct;
                acc.q_y(i) = acc.q_y(i) + dq(2)*Ct;
                acc.q_z(i) = acc.q_z(i) + dq(3)*Ct;
                acc.q_w(i) = acc.q_w(i) + dq(4)*Ct;

                % --- 2nd -> 1st ---
                % Velocities (linear)
                acc.v_x(i) = acc.v_x(i) + state.a_x(i)*Ct;
                acc.v_y(i) = acc.v_y(i) + state.a_y(i)*Ct;
                acc.v_z(i) = acc.v_z(i) + state.a_z(i)*Ct;
                % Velocities (angular)
                acc.w_x(i) = acc.w_x(i) + state.dw_x(i)*Ct;
                acc.w_y(i) = acc.w_y(i) + state.dw_y(i)*Ct;
                acc.w_z(i) = acc.w_z(i) + state.dw_z(i)*Ct;
            end

            % X - [p;v;a]
            v_0 = X0(4:6,1);
            a_0 = X0(7:9,1);
            % Constraint integration
            for i = i:state.NumberOfConstraints
                acc.r_x(i) = acc.r_x(i) + state.r_x(i) * Ct;
                acc.r_y(i) = acc.r_x(i) + state.r_y(i) * Ct;
                acc.r_t(i) = acc.r_x(i) + state.r_t(i) * Ct;
            end

            da = zeros(3,1);
            dv = a_0*dt;
            dp = v_0 + dv;
            % Collect the differentials
            dX = [dp;dv;da];
            % Write the accumulated terms to the state object in final stage
            if this.Stage == 4
                % The states
                for i = 1:state.NumberOfBodies
                    % --- 1st -> 0th ---
                    % Positions (linear)
                    state.p_x(i) = acc.p_x(i);
                    state.p_y(i) = acc.p_y(i);
                    state.p_z(i) = acc.p_z(i);
                    % Positions (angular)
                    state.q_x(i) = acc.q_x(i);
                    state.q_y(i) = acc.q_y(i);
                    state.q_z(i) = acc.q_z(i);
                    state.q_w(i) = acc.q_w(i);

                    % --- 2nd -> 1st ---
                    % Velocities (linear)
                    state.v_x(i) = acc.v_x(i);
                    state.v_y(i) = acc.v_y(i);
                    state.v_z(i) = acc.v_z(i);
                    % Velocities (angular)
                    state.w_x(i) = acc.w_x(i);
                    state.w_y(i) = acc.w_y(i);
                    state.w_z(i) = acc.w_z(i);
                end
                % The constraints
                for i = 1:state.NumberOfConstraints
                    state.r_x(i) = acc.r_x(i);
                    state.r_y(i) = acc.r_y(i);
                    state.r_z(i) = acc.r_z(i);
                    state.t_x(i) = acc.t_x(i);
                    state.t_y(i) = acc.t_y(i);
                    state.t_z(i) = acc.t_z(i);
                end
            end

            % Move to next stage
            this.Stage = this.Stage + 1;

            % Solve again
            if (this.Stage < 5)
                state = this.Solve(state);
            end
        end
        % End the current step
        function [this] = End(this)
            this.Stage = -1;
        end
    end

    methods (Static)
        function [X] = IntegrateFunWithInputs(fun,dt,t0,X0,U0)
            % This function provide the raw method, with inputs

            % Calculate the RK4 gains
            k1 = fun(t0,X0,U0);
            k2 = fun(t0+dt/2,X0+k1*dt/2,U0);
            k3 = fun(t0+dt/2,X0+k2*dt/2,U0);
            k4 = fun(t0+dt,X0+k3*dt,U0);
            % The next state
            X = X0 +(k1/6+k2/3+k3/3+k4/6)*dt;
        end
        function [X] = IntegrateFun(fun,dt,t0,X0)
            % This function provides the raw method.

            % Calculate the RK4 gains
            k1 = fun(t0,X0);
            k2 = fun(t0+dt/2,X0+k1*dt/2);
            k3 = fun(t0+dt/2,X0+k2*dt/2);
            k4 = fun(t0+dt,X0+k3*dt);
            % The next state
            X = X0 +(k1/6+k2/3+k3/3+k4/6)*dt;
        end
        function [this] = IntegrateTransform(this,transformation,dt)
            % This function computes the RK4 method against a given
            % transform to integrate its state-change over-time.

            % Get the initial frame-state
            p0 = transformation.GetWorldPosition();
            v0 = transformation.Velocity;
            a0 = transformation.Acceleration;

            % Wrapper
            X0 = [p0;v0;a0];
            % Integrate
            dX = this.IntegrateFun(@RK4Integrator.MotionStep,dt,0,X0);
            % [SOMETHING IS WRONG HERE
            X = X0 + dX*dt;
            % Reapply
            transformation.SetWorldPosition(X(1:3,1));
            transformation.Velocity = X(4:6,1);
            transformation.Acceleration = X(7:9,1);
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
