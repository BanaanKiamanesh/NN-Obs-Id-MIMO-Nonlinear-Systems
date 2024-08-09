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
        
        % Size of the V and W Matrices
        VSize
        WSize

        eta1 = 50000
        rho1 = 0.03    
        eta2 = 10000
        rho2 = 0.1
    end

    methods
        function Obj = Attractor
            Obj.G = place(Obj.A', Obj.C', Obj.DesiredPoles)';  % Luenberger Observer
            Obj.Ac = Obj.A - Obj.G*Obj.C;

            % Init W and V
            Obj.VSize = [Obj.SystemOrder, Obj.NumNeurons];
            Obj.WSize = [Obj.SystemOrder, Obj.NumNeurons];
        end

        function dX = SysDyn(Obj, X, ~)
            % State Unpack
            X1 = X(1);
            X2 = X(2);
            X3 = X(3);

            dX = zeros(3, 1); 
            dX(1) = Obj.sigma * (X2 - X1);
            dX(2) = X1 * (Obj.rho - X3) - X2;
            dX(3) = X1 * X2 - Obj.beta * X3;
        end

        function dX_hat = Observer(Obj, X_hat, Xerr, W, V)
            %%%% Observer: X_hat =   A*X_hat   +    g_hat   +   G*(y - y_hat)
            %%%%                   ---Term1---  ---Term2---   -----Term3-----

            % Term 1 Calc
            Term1 = Obj.A * X_hat;

            % Term 2 Calc (Neural Network) (g_hat)
            Term2 = W * logsig(V' * X_hat);

            % Term 3 Calc
            Term3 = Obj.G * Xerr;

            dX_hat = Term1 + Term2 + Term3;
        end

        function [dW, dV] = LearningRule(Obj, W, V, Xerr, X_hat)
            % W dot
            dW = -Obj.eta1 * ((Xerr' * Obj.C * inv(Obj.Ac))') * (logsig(V' * X_hat)') - Obj.rho1 * norm(Xerr) * W;
        
            % V dot
            Lambda = diag(logsig(V' * X_hat).^2);
            dV = -Obj.eta2 * ( ((Xerr' * Obj.C * inv(Obj.Ac) * W * (eye(size(Lambda)) - Lambda))') * (sign(X_hat)'))' - Obj.rho2 * norm(Xerr) * V;
        
        end

        function dX_out = ODE(Obj, Xin, t)
            % Unpack States
            X     = Xin(1:Obj.SystemOrder); 
            X_hat = Xin(Obj.SystemOrder + 1:2 * Obj.SystemOrder);  
            W = reshape(Xin(2*Obj.SystemOrder + 1:end - prod(Obj.VSize)), Obj.WSize);
            V = reshape(Xin(end - prod(Obj.VSize) + 1:end), Obj.VSize);
            
            % Estimation Err
            X_Err = X - X_hat;

            dX = Obj.SysDyn(X, t);
            dX_hat = Obj.Observer(X_hat, X_Err, W, V);
            [dW, dV] = Obj.LearningRule(W, V, X_Err, X_hat);

            dX_out = [dX; dX_hat; dW(:); dV(:)];
        end

        function Motion = Simulate(Obj, SimTime)
            % ODE Options
            ODEFun = @(t, X) Obj.ODE(X, t);
            tSpan = [0 SimTime];

            % Use Plotter for State Vs Estimated
            % Use Plotter2 for Estimation Error
            Opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-9, 'OutputFcn', @Plotter);
            X0 = 10 * rand(2*Obj.SystemOrder + prod(Obj.WSize) + prod(Obj.VSize), 1);

            [Motion.t, Motion.y] = ode45(ODEFun, tSpan, X0, Opt);
        end
    end
end