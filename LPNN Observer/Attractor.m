classdef Attractor
    properties
        % System Dynamical Properties
        sigma = 10.0
        rho = 28.0
        beta = 8/3

        % Observer Dynamics Matrix
        A = -20 * eye(3)

        C = eye(3);                       % System Output Matrix
        DesiredPoles = -100 * ones(1, 3)  % Desired Poles for The Observer
        G                                 % Observer Gain
        Ac                                % (A - GC) X_err

        % Neural Network Properties
        NumNeurons = 10
        SystemOrder = 3
        V

        eta = 50000
        rho_l = 0.03
    end

    methods
        function Obj = Attractor
            Obj.G = place(Obj.A', Obj.C', Obj.DesiredPoles)';  % Luenberger Observer
            Obj.Ac = Obj.A - Obj.G*Obj.C;

            % Init W and V
            Obj.V = eye(Obj.SystemOrder, Obj.NumNeurons);
        end

        function dX = SysDyn(Obj, X, t)
            % State Unpack
            X1 = X(1);
            X2 = X(2);
            X3 = X(3);

            dX = zeros(3, 1);
            dX(1) = Obj.sigma * (X2 - X1);
            dX(2) = X1 * (Obj.rho - X3) - X2;
            dX(3) = X1 * X2 - Obj.beta * X3;
        end

        function dX_hat = Observer(Obj, X_hat, Xerr, W)
            %%%% Observer: X_hat =   A*X_hat   +    g_hat   +   G*(y - y_hat)
            %%%%                   ---Term1---  ---Term2---   -----Term3-----

            % Term 1 Calc
            Term1 = Obj.A * X_hat;

            % Term 2 Calc (Neural Network) (g_hat)
            Term2 = W * logsig(Obj.V' * X_hat);

            % Term 3 Calc
            Term3 = Obj.G * Xerr;

            dX_hat = Term1 + Term2 + Term3;
        end

        function dW = LearningRule(Obj, W, Xerr, X_hat)
            dW = -Obj.eta * ((Xerr' * Obj.C' * Obj.C * inv(Obj.Ac))') * (logsig(Obj.V' * X_hat)') - Obj.rho_l * norm(Xerr) * W;
        end

        function dX_out = ODE(Obj, Xin, t)
            % Unpack States
            X     = Xin(1:3);
            X_hat = Xin(4:6);
            W = reshape(Xin(7:end), [Obj.SystemOrder, Obj.NumNeurons]);

            % Estimation Err
            X_Err = X - X_hat;

            dX = Obj.SysDyn(X, t);
            dX_hat = Obj.Observer(X_hat, X_Err, W);
            dW = Obj.LearningRule(W, X_Err, X_hat);

            dX_out = [dX; dX_hat; dW(:)];
        end

        function Motion = Simulate(Obj, SimTime)
            % ODE Options
            ODEFun = @(t, X) Obj.ODE(X, t);
            tSpan = [0 SimTime];

            % Use Plotter for State Vs Estimated
            % Use Plotter2 for Estimation Error
            Opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-9, 'OutputFcn', @Plotter);
            X0 = 10 * rand(2*Obj.SystemOrder + Obj.SystemOrder * Obj.NumNeurons, 1);

            [Motion.t, Motion.y] = ode45(ODEFun, tSpan, X0, Opt);
        end
    end
end