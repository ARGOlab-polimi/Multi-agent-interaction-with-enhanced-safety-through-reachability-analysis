function V_H(t, V, limiti, U_safe, V_threshold, onoff, t_start, t_end)

% Filter data based on the time range
time_mask = t >= t_start & t <= t_end;
t = t(time_mask) - t_start; % Shift time to start from zero
V = V(time_mask);
limiti = limiti(time_mask);
U_safe = U_safe(time_mask);
onoff = onoff(time_mask);

% figure creation
figure;


plot(t, V, 'LineWidth', 2, 'Color', [0 0.447 0.741]) % Smooth, high-contrast color
hold on
yline(V_threshold, 'k', 'LineWidth', 2) % Dashed red line for threshold

% Labels and title with enhanced fonts
xlabel('$t\ [s]$', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'Latex')
ylabel('$V$', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'Latex')

% Improve axis aesthetics
ax = gca;
ax.FontSize = 12; % Adjust axis font size
ax.FontWeight = 'Bold';
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridLineStyle = '--'; % Dashed grid
ax.LineWidth = 1.2; % Thicker axis lines
set(gcf, 'Position', [100, 100, 600, 300]) % Wider figure
legend({'Value Function', 'Threshold'}, 'Location', 'Best', 'FontSize', 12, 'Interpreter', 'Latex', 'Box', 'off')

% Export settings for high-quality output
set(gcf, 'Color', 'w'); % White background for a clean look
set(gca, 'LooseInset', max(get(gca, 'TightInset'), 0.02)) % Prevents axis cropping

yline(0, "LineWidth", 0.2)                                                  % 0 line
xlim([t(1), t(end)]);
ax=gca;
ylim([ax.YLim(1) 10])
hold on

% Define the x-limits of the regions to highlight
prec = 0;
x_regions = [];
for i = 1:length(limiti)
    if limiti(i) ~= prec    
        x_regions = [x_regions, i];
        prec = not(prec);
    end
end

% Get current y-limits of the plot
yl = ylim;

% Highlight green regions based on `limiti`
for i = 1:length(x_regions)/2
    x_start = t(x_regions(2*i-1));
    x_end = t(x_regions(2*i));
    
    % Create a green patch for each region
    patch([x_start x_start x_end x_end], [yl(1) yl(2) yl(2) yl(1)], ...
          'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end

% Highlight red regions based on `onoff` and `limiti`
for i = 1:length(t)
    if limiti(i) == 1 && onoff(i) == 1
        % Define the x-range for this region
        x_start = t(i);
        if i < length(t)
            x_end = t(i+1);
        else
            x_end = t(i); % Last point
        end

        % Create a red patch for `onoff` regions
        patch([x_start x_start x_end x_end], [yl(1) yl(2) yl(2) yl(1)], ...
              'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
end


figure;
plot(t, U_safe, 'LineWidth', 2, 'Color', [0 0.447 0.741]) % Smooth, high-contrast color
hold on

% Labels and title with enhanced fonts
xlabel('$t\ [s]$', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'Latex')
ylabel('$H$', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'Latex')

% Improve axis aesthetics
ax = gca;
ax.FontSize = 12; % Adjust axis font size
ax.FontWeight = 'Bold';
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridLineStyle = '--'; % Dashed grid
ax.LineWidth = 1.2; % Thicker axis lines
% ax.Box = 'on'; % Keep the box for a structured look
set(gcf, 'Position', [100, 100, 600, 300]) % Wider figure

% Export settings for high-quality output
set(gcf, 'Color', 'w'); % White background for a clean look
set(gca, 'LooseInset', max(get(gca, 'TightInset'), 0.02)) % Prevents axis cropping

yline(0, "LineWidth", 0.2)                                                  % 0 line
xlim([t(1), t(end)]);