function status = Plotter2(t, y, flag)
    persistent figHandle trueLine3D estLine3D errLines
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

        errLines = gobjects(1, numStates);

        for i = 1:numStates
            subplot(3, 2, 2*i);
            hold on;
            grid on;
            errLines(i) = animatedline('Color', 'k', 'LineStyle', '-.', 'DisplayName', ['$e_', num2str(i), '$']);
            xlabel('Time');
            ylabel(['Error e_', num2str(i)]);
            legend('show', 'Interpreter', 'latex', 'FontSize', 12);
            legend('boxoff');
        end
        sgtitle("Lorenz Attractor and Estimation Errors", 'FontWeight', 'bold', 'FontSize', 20);

        status = 0;
        return;
    end

    if isempty(flag)  % Solver State Saying it is Going!
        trueStates = y(1:3);
        estStates = y(4:6);
        errors = trueStates - estStates;

        addpoints(trueLine3D, trueStates(1), trueStates(2), trueStates(3));
        addpoints(estLine3D, estStates(1), estStates(2), estStates(3));

        for i = 1:numStates
            addpoints(errLines(i), t(end), errors(i));
        end
        drawnow limitrate;
    end

    if strcmp(flag, 'done') % Solver State Saying it is Done!
        status = 0;
        return;
    end

    status = 0;
end