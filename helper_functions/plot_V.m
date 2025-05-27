% function plot_V(t, V, limiti, U_safe, V_threshold, num_vehicle, onoff, h, x0_ego)
% 
% % figure creation
% figure;
% subplot(211)
% plot(t, V)
% xlabel('t')
% ylabel('V')
% grid on
% yline(0, "LineWidth", 0.2)                                                  % 0 line
% yline(V_threshold, "LineWidth", 0.6)                                        % threshold line
% xlim([t(1), t(end)]);
% hold on
% title(['Value function wrt vehicle ', num2str(num_vehicle)])
% 
% % Define the x-limits of the regions to highlight
% prec = 0;
% x_regions = [];
% for i = 1:length(limiti)
%     if limiti(i) ~= prec    
%         x_regions = [x_regions, i];
%         prec = not(prec);
%     end
% end
% 
% % Get current y-limits of the plot
% yl = ylim;
% 
% % Highlight green regions based on `limiti`
% for i = 1:length(x_regions)/2
%     x_start = t(x_regions(2*i-1));
%     x_end = t(x_regions(2*i));
% 
%     % Create a green patch for each region
%     patch([x_start x_start x_end x_end], [yl(1) yl(2) yl(2) yl(1)], ...
%           'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
% end
% 
% % Highlight red regions based on `onoff` and `limiti`
% for i = 1:length(t)
%     if limiti(i) == 1 && onoff(i) == 1
%         % Define the x-range for this region
%         x_start = t(i);
%         if i < length(t)
%             x_end = t(i+1);
%         else
%             x_end = t(i); % Last point
%         end
% 
%         % Create a red patch for `onoff` regions
%         patch([x_start x_start x_end x_end], [yl(1) yl(2) yl(2) yl(1)], ...
%               'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
%     end
% end
% 
% do_thr = false;
% if do_thr 
% % Highlight yellow regions if limitCount >= 2
% for i = 1:length(t)
%     limitCount = 0; % Initialize the counter for each time step
%     for ii = 1:length(h)
%         if h{ii}.limiti(i) == 1 && ...
%            h{ii}.V(i) < h{ii}.settings.V_threshold + h{ii}.simX_0(4)/2.5 + ...
%            abs(x0_ego(i,5) - h{ii}.simX_0(4)*cos(h{ii}.simX_0(3))) %* (h{ii}.simX_0(2) > 0)
%             limitCount = limitCount + 1;
%         end
%     end
% 
%     % Check if the condition is satisfied
%     if limitCount >= 2
%         % Define the x-range for this region
%         x_start = t(i);
%         if i < length(t)
%             x_end = t(i+1);
%         else
%             x_end = t(i); % Last point
%         end
% 
%         % Create a yellow patch for the region
%         patch([x_start x_start x_end x_end], [yl(1) yl(2) yl(2) yl(1)], ...
%               'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
%     end
% end
% 
% % Plot the dynamic threshold line for `num_vehicle`
% dynamic_threshold = h{num_vehicle}.settings.V_threshold + h{num_vehicle}.simX_0(4)/2.5 + ...
%                     abs(x0_ego(:,5) - h{num_vehicle}.simX_0(4) * cos(h{num_vehicle}.simX_0(3))) .* ...
%                     (h{num_vehicle}.simX_0(2) > 0);
% plot(t, dynamic_threshold, '--', 'LineWidth', 1, 'Color', 'm', 'DisplayName', 'Dynamic Threshold');
% % legend show;
% end
% 
% subplot(212)
% plot(t, U_safe);
% xlabel('t');
% ylabel('H');
% title(['Hamiltonian wrt vehicle ', num2str(num_vehicle)])
% grid;
% xlim([t(1), t(end)]);
% end

function plot_V(t, V, limiti, U_safe, V_threshold, num_vehicle, onoff, h, x0_ego, t_start, t_end)

% Filter data based on the time range
time_mask = t >= t_start & t <= t_end;
t = t(time_mask) - t_start; % Shift time to start from zero
V = V(time_mask);
limiti = limiti(time_mask);
U_safe = U_safe(time_mask);
onoff = onoff(time_mask);

% figure creation
figure;
subplot(211)
plot(t, V)
xlabel('t [s]')
ylabel('V')
grid on
yline(0, "LineWidth", 0.2)                                                  % 0 line
yline(V_threshold, "LineWidth", 0.6)                                        % threshold line
xlim([t(1), t(end)]);
hold on
title(['Value function wrt vehicle ', num2str(num_vehicle)])

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

do_thr = false;
if do_thr 
% Highlight yellow regions if limitCount >= 2
for i = 1:length(t)
    limitCount = 0; % Initialize the counter for each time step
    for ii = 1:length(h)
        if h{ii}.limiti(i) == 1 && ...
           h{ii}.V(i) < h{ii}.settings.V_threshold + h{ii}.simX_0(4)/2.5 + ...
           abs(x0_ego(i,5) - h{ii}.simX_0(4)*cos(h{ii}.simX_0(3))) %* (h{ii}.simX_0(2) > 0)
            limitCount = limitCount + 1;
        end
    end
    
    % Check if the condition is satisfied
    if limitCount >= 2
        % Define the x-range for this region
        x_start = t(i);
        if i < length(t)
            x_end = t(i+1);
        else
            x_end = t(i); % Last point
        end
        
        % Create a yellow patch for the region
        patch([x_start x_start x_end x_end], [yl(1) yl(2) yl(2) yl(1)], ...
              'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
end

% Plot the dynamic threshold line for `num_vehicle`
dynamic_threshold = h{num_vehicle}.settings.V_threshold + h{num_vehicle}.simX_0(4)/2.5 + ...
                    abs(x0_ego(:,5) - h{num_vehicle}.simX_0(4) * cos(h{num_vehicle}.simX_0(3))) .* ...
                    (h{num_vehicle}.simX_0(2) > 0);
plot(t, dynamic_threshold, '--', 'LineWidth', 1, 'Color', 'm', 'DisplayName', 'Dynamic Threshold');
% legend show;
end

subplot(212)
plot(t, U_safe);
xlabel('t [s]');
ylabel('H');
title(['Hamiltonian wrt vehicle ', num2str(num_vehicle)])
grid;
xlim([t(1), t(end)]);
end
