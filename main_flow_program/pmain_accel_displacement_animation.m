

%% =========================== 2D Animation ============================ %%
if INI.visualize.animatedDisplacement2DPlot
    
	% ------ INPUT DATA ------ %
    timeSamples = time;                     % raw data time samples
    dataToBeDisplayed = displFilt;          % data for animation
    
	
    figure('units', 'normalized', 'outerposition', [0.3 0.2 .4 .7], 'IntegerHandle', 'off', ...
           'Name', '2D Animated Displacement plot');
    curve = animatedline('LineWidth', 2);
    set(gca, 'XLim', [-1 1], 'YLim', [-1 1]);
    set(gca, 'SortMethod', 'depth');
    set(gcf, 'Renderer', 'zbuffer');
    hold on;  grid on;    
    
    SamplePlotFreq = 8;                     % time samples increment for displayings
    [newPlotSamplesN, ~] = size(timeSamples(1:SamplePlotFreq:length(timeSamples), 1));
    
    for t=1:newPlotSamplesN
        curIndex = 1 + (t-1)*SamplePlotFreq;
        % Update graph title
        titleText = sprintf('Sample %i of %i', t, newPlotSamplesN);
        title(titleText);
        
        addpoints(curve, dataToBeDisplayed(curIndex,1), dataToBeDisplayed(curIndex,2));
        curPoint = scatter(dataToBeDisplayed(curIndex,1), dataToBeDisplayed(curIndex,2), ...
            'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
        drawnow
        pause(0.002);
        delete(curPoint);
    end        
end



%% =========================== 3D Animation ============================ %%
if INI.visualize.animatedDisplacement3DPlot    
  
    % ------ INPUT DATA ------ %
    SamplePlotFreq = 8;                     % time samples increment for displaying
    dataToBeAnimated = displFilt;           % data for animation
    SamplePeriod = TS;                      % raw data sample period
    rotationMatrix = RM;                    % rotation matrix

    
    SixDOFanimation(dataToBeAnimated, rotationMatrix, ...
        'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
        'Position', [9 39 1280 720], ...
        'AxisLength', 1, 'ShowArrowHead', false, ...
        'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
        'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/SamplePeriod) / SamplePlotFreq)); 
end


