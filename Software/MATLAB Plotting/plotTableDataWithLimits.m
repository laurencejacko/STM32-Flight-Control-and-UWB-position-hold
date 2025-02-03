function plotTableDataWithLimits(tableData, col_ind, yLimits, xlineslims,fignum)
    % Function to plot specified columns of a table with custom y-axis limits
    %
    % Inputs:
    %   - tableData: Table containing the data to plot
    %   - startCol: Starting column index to plot
    %   - endCol: Ending column index to plot
    %   - yLimits: 2-element vector specifying [yMin yMax] for y-axis limits
    %
    % Example:
    %   plotTableDataWithLimits(PositionErr, 2, 4, [-10 10])

    % Validate inputs
    % if startCol < 1 || endCol > width(tableData)
    %     error('Column indices are out of range.');
    % end
    if length(yLimits) ~= 2
        error('yLimits must be a 2-element vector [yMin yMax].');
    end
    
    % Extract the column range and corresponding names
    selectedData = tableData(:, col_ind);
    selectedNames = selectedData.Properties.VariableNames;

    % Plot the data
 %   figure(fignum);
    hold on;
    box on
    for i = 1:width(selectedData)
        plot( selectedData{:, i}); % Plot each selected column
    end
   
    dt = 20e-3;
 hold off;
    % Customize plot
    legend(string(selectedNames), 'Interpreter', 'latex','AutoUpdate','off');
    xlabel('Sample Index'); % Label for x-axis
    ylabel('Value'); % Label for y-axis
    title('Selected Table Data: Sample Time = '+string(dt)+'s');
    ylim(yLimits); % Apply y-axis limits
    xline(xlineslims,'LineWidth',1 ,'HandleVisibility', 'off'); % Exclude xline from legend    

    grid on;
end
