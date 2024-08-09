function status = Plotter(t, y, flag)
    persistent figHandle trueLine3D estLine3D StateLines EstLines
    numStates = 3;

    if strcmp(flag, 'init') % Solver State Saying it is Started Solving!
        figHandle = figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);

        subplot(1, 2, 1);
        hold on;
        grid on;
        trueLine3D = animatedline('Color', 'k', 'LineWidth', 1.5, 'DisplayName', 'True Attractor');
        estLine3D = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'Estimated Attractor');
        view(-5, 0);
        xlabel('x_1');
        ylabel('x_2');
        zlabel('x_3');
        title('Lorenz Attractor and Estimated Trajectory');
        legend('show', 'Interpreter', 'latex', 'FontSize', 12);
        legend('boxoff');
        axis off

        StateLines = gobjects(1, numStates);
        EstLines   = gobjects(1, numStates);

        for i = 1:numStates
            subplot(3, 2, 2*i);
            hold on;
            grid on;
            StateLines(i) = animatedline('Color', 'k', 'LineStyle', '--', 'DisplayName', ['$x_', num2str(i), '$']);
            EstLines(i)   = animatedline('Color', 'r', 'LineStyle', '-.', 'DisplayName', ['$\hat{x}_', num2str(i), '$']);
            xlabel('Time');
            ylabel(['Error e_', num2str(i)]);
            legend('show', 'Interpreter', 'latex', 'FontSize', 20);
        end
        sgtitle("Lorenz Attractor Neural Observer", 'FontWeight', 'bold', 'FontSize', 20);

        status = 0;
        return;
    end

    if isempty(flag)  % Solver State Saying it is Going!
        trueStates = y(1:3);
        estStates = y(4:6);

        addpoints(trueLine3D, trueStates(1), trueStates(2), trueStates(3));
        addpoints(estLine3D, estStates(1), estStates(2), estStates(3));

        % Add points to the error plots
        for i = 1:numStates
            addpoints(StateLines(i), t(end), trueStates(i));
            addpoints(EstLines(i), t(end), estStates(i));
        end
        drawnow limitrate;
    end

    if strcmp(flag, 'done') % Solver State Saying it is Done!
        status = 0;
        return;
    end

    status = 0;
end