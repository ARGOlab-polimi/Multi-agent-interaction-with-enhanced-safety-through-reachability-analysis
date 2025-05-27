function plot_full_trajectories(t, simX, h, track_file, diff_color, lane_width, tstart, tend)

% Filter data within the specified time range
valid_indices = t >= tstart & t <= tend;
t = t(valid_indices) - tstart; % Shift time to start at 0
simX = simX(valid_indices, :);
for v = 1:length(h)
    h{v}.simX = h{v}.simX(valid_indices, :);
end

% Load ego states
s = simX(:,1);
n = simX(:,2);
theta = simX(:,3);
delta = simX(:,4);
v = simX(:,5);

% Transform data
[x, y, ~, ~] = transformProj2Orig(s, n, theta, v, track_file);

% Get track
[~, Xref, Yref, Psiref, ~] = getTrack(track_file);
y_lim = lane_width; % Road lane width

% Create a figure and set it to full screen
figure();
axis tight;
hold on;
grid on;

% Define two colormaps
if diff_color
    ego_colors = autumn(length(t));         % Color map for the ego vehicle
    hdv_colors = jet(length(t));           % Color map for the HDVs
else
    ego_colors = jet(length(t));
    hdv_colors = jet(length(t));
end

% Plot ego trajectory with time-colored evolution
for i = 1:length(t)-1
    plot([x(i), x(i+1)], [y(i), y(i+1)], 'Color', ego_colors(i, :), 'LineWidth', 1.5);
    hold on;
end

% Plot road boundaries
Xboundleft = Xref - y_lim * sin(Psiref);
Yboundleft = Yref + y_lim * cos(Psiref);
Xboundright = Xref + y_lim * sin(Psiref);
Yboundright = Yref - y_lim * cos(Psiref);
hold on;
plot(Xboundleft, Yboundleft, 'k', 'LineWidth', 1);
plot(Xboundright, Yboundright, 'k', 'LineWidth', 1);

% Title and labels
xlabel('$X\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$Y\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)

% Place small markers at every 1-second interval
indices = [];
multiples_of_one = 0:floor(max(t));
for i = 1:length(multiples_of_one)
    idx = find(t >= multiples_of_one(i), 1);
    if ~isempty(idx) && (isempty(indices) || idx > indices(end))
        indices = [indices, idx];
    end
end

% Plot small markers at regular intervals on ego trajectory
scatter(x(indices), y(indices), 5, 'b', 'filled');

for v = 1:length(h)
    % Load HDV states
    s2 = h{v}.simX(:,1);
    n2 = h{v}.simX(:,2);
    theta2{v} = h{v}.simX(:,3);
    v2 = h{v}.simX(:,4);

    % Transform data
    [x2{v}, y2{v}, ~, ~] = transformProj2Orig(s2, n2, theta2{v}, v2, track_file);

    for i = 1:length(t)-1
        plot([x2{v}(i), x2{v}(i+1)], [y2{v}(i), y2{v}(i+1)], 'Color', hdv_colors(i, :), 'LineWidth', 1.5);
        hold on;
    end

    % Plot small markers at regular intervals on both trajectories
    scatter(x2{v}(indices), y2{v}(indices), 5, 'r', 'filled');
end

xlim([min(x), max(x)]);
ylim([-y_lim-0.5, y_lim+0.5]);

% Set aspect ratio to make the plot rectangular
pbaspect([2 1 1]);

% Add colorbar for the ego vehicle
if diff_color
    c1 = colorbar('Location', 'eastoutside'); % Right side for ego vehicle
    c1.Label.String = 'Time (Ego Vehicle)';
    colormap(c1, 'autumn');  % For ego vehicle
    caxis([min(t) max(t)]); % Set color limits

    c1.Label.Interpreter = 'latex';
c1.Label.String = '$\text{Time} \ [s]$';
c1.Label.FontSize = 14;

    % Add colorbar for HDVs
    c2 = colorbar('Location', 'westoutside'); % Left side for HDVs
    c2.Label.String = 'Time (HDVs)';
    colormap(c2, 'jet');  % For HDVs
    caxis([min(t) max(t)]); % Set color limits

else
    c1 = colorbar('Location', 'eastoutside'); % Right side for ego vehicle
    colormap(c1, 'jet');  % For ego vehicle
    caxis([min(t) max(t)]); % Set color limits
    c1.Label.Interpreter = 'latex';
    c1.Label.String = '$t \ [s]$';
    c1.Label.FontSize = 14;

end

%% Velocity

% Load ego states
s = simX(:,1);
n = simX(:,2);
theta = simX(:,3);
delta = simX(:,4);
v = simX(:,5);

% Transform data
[x, y, ~, ~] = transformProj2Orig(s, n, theta, v, track_file);

% Get track
[~, Xref, Yref, Psiref, ~] = getTrack(track_file);
y_lim = lane_width; % Road lane width

% Create a figure for velocity-colored trajectories
figure;
axis tight;
hold on;
grid on;
% Title and labels
xlabel('$X\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$Y\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)


% Define color map for velocity
velocity_colors = (jet(256)); % Color map for velocity
global_v_min = 1; % Minimum velocity (global)
global_v_max = 11; % Maximum velocity (global)

% Plot HDVs (velocity-colored)
for v_idx = 1:length(h)
    v2 = h{v_idx}.simX(:,4); % Velocity of the current HDV

    % Normalize velocity using the global range [1, 11]
    normalized_v2 = (v2 - global_v_min) / (global_v_max - global_v_min) * (size(velocity_colors, 1) - 1) + 1;

    % Transform HDV coordinates
    [x2, y2, ~, ~] = transformProj2Orig(h{v_idx}.simX(:,1), h{v_idx}.simX(:,2), h{v_idx}.simX(:,3), v2, track_file);

    % Plot HDV trajectory
    for i = 1:length(t)-1
        vel_idx = round(normalized_v2(i)); % Find the colormap index
        vel_idx = max(1, min(vel_idx, size(velocity_colors, 1))); % Ensure index is within bounds
        vel_color = velocity_colors(vel_idx, :); % Get the color
        plot([x2(i), x2(i+1)], [y2(i), y2(i+1)], 'Color', vel_color, 'LineWidth', 1.5);
    end
end

% Plot road boundaries
Xboundleft = Xref - y_lim * sin(Psiref);
Yboundleft = Yref + y_lim * cos(Psiref);
Xboundright = Xref + y_lim * sin(Psiref);
Yboundright = Yref - y_lim * cos(Psiref);
plot(Xboundleft, Yboundleft, 'k', 'LineWidth', 1);
plot(Xboundright, Yboundright, 'k', 'LineWidth', 1);

% Plot ego trajectory (velocity-colored) last to ensure visibility
normalized_v = (v - global_v_min) / (global_v_max - global_v_min) * (size(velocity_colors, 1) - 1) + 1;

for i = 1:length(t)-1
    vel_idx = round(normalized_v(i)); % Find the colormap index
    vel_idx = max(1, min(vel_idx, size(velocity_colors, 1))); % Ensure index is within bounds
    vel_color = velocity_colors(vel_idx, :); % Get the color
    plot([x(i), x(i+1)], [y(i), y(i+1)], 'Color', vel_color, 'LineWidth', 2.5, 'LineStyle', '--'); % Ego with thicker dashed line
end

xlim([min(x), max(x)]);
ylim([-y_lim-0.5, y_lim+0.5]);
pbaspect([2 1 1]);

% Add colorbar for velocity
cbar_velocity = colorbar('eastoutside'); % Create the colorbar
colormap(cbar_velocity, 'jet'); % Use parula colormap for velocity
caxis([global_v_min global_v_max]); % Use global velocity range for the colorbar
cbar_velocity.Label.Interpreter = 'latex';
cbar_velocity.Label.String = '$Velocity\ [m/s]$';
cbar_velocity.Label.FontSize = 14;
end