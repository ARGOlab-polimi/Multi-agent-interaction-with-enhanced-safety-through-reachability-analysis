function plot_trajectories(t, simX, h, track_file, model, traj_setting, BRT_small,  ...
    BRT_big, lane_width,save_str, manual_time)



% Load BRT data
load(BRT_small);
V_small = BRT.data;
clear BRT

load(BRT_big);
V_big = BRT.data;
clear BRT

% Load settings
animate = traj_setting.do_animate;
anim_rate = traj_setting.anim_rate;                                                      
dovideo = traj_setting.do_video;                                                         
do_BRT = traj_setting.do_BRT_plot;    

% Setup for video making
if dovideo

    video_count = 1;

    while true

        name = ['Overtake_lin_ls_',num2str(video_count)];
        name = strcat(name,'.mp4');

        % Checks how many videos are already present
        if ~exist(name,"file")
            break
        else
            video_count = video_count+1;
        end
    end

    writerObj = VideoWriter(name, 'MPEG-4');
    writerObj.Quality = 100;
    writerObj.FrameRate = 30;
    open(writerObj);
end

% Vehicle dimensions 
D1 = model.params.D1-0.2*2; 
D2 = model.params.D2-0.2*2 ;
D1_h = model.params.D1_h; 
D2_h = model.params.D2_h; 

% Wheel dimensions (suppposed dimensions)
wheel_l = 1; % meters
wheel_w = 0.2; % meters

% Load ego states
s = simX(:,1);
n = simX(:,2);
theta = simX(:,3);
delta = simX(:,4);
v = simX(:,5);

% Transform data
[x, y, ~, ~] = transformProj2Orig(s, n, theta, v, track_file);

% Get track
[~,Xref,Yref,Psiref,~] = getTrack(track_file);
y_lim = lane_width; % road lane width


% Create a figure
figure;
axis equal
hold on;
grid on;


% Plot road boundaries
Xboundleft = Xref - y_lim * sin(Psiref);
Yboundleft = Yref + y_lim * cos(Psiref);
Xboundright = Xref + y_lim * sin(Psiref);
Yboundright = Yref - y_lim * cos(Psiref);
hold on;
plot(Xboundleft-20, Yboundleft, 'k', 'LineWidth', 1);
plot(Xboundright-20, Yboundright, 'k', 'LineWidth', 1);

% Title and labels
xlabel('$X\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$Y\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)


for v = 1:length(h)

    % Load HDV states
    s2{v} = h{v}.simX(:,1);
    n2{v} = h{v}.simX(:,2);
    theta2{v} = h{v}.simX(:,3);
    delta2{v} = h{v}.simX(:,3); % Steering angle for vehicle 2 (TO BE MODIFIED, now just equal to yaw)
    v2{v} = h{v}.simX(:,4);

    % Transform data
    [x2{v}, y2{v}, ~, ~] = transformProj2Orig(s2{v}, n2{v}, theta2{v}, v2{v}, track_file);

end

% Initialize cars drawings objects
ego_shape = [];
ego_wheel_shape = [];



for v = 1:length(h)
    hdv_shape{v} = [];
    hdv_wheel_shape{v} = [];
    V_0{v} = [];
end

% Plot trajectories
traj_ego = plot(x(1), y(1), 'b--', 'LineWidth', 1); % trajectory of vehicle 1
traj_human1 = plot(x2{1}(1), y2{1}(1), 'r--', 'LineWidth', 1); % trajectory of vehicle 2

if length(h)>1
    traj_human2 = plot(x2{2}(1), y2{2}(1), 'r--', 'LineWidth', 1); % trajectory of vehicle 2
    traj_human2_x = x2{2}(1);  % Store initial human position
    traj_human2_y = y2{2}( 1);  % Store initial human position
end

% Store past trajectory points
traj_ego_x = x(1);  % Store initial ego position
traj_ego_y = y(1);  % Store initial ego position
traj_human1_x = x2{1}(1);  % Store initial human position
traj_human1_y = y2{1}( 1);  % Store initial human position


background_V = [];
x_center = x(1); % Ego vehicle's x position
y_center = y(1); % Ego vehicle's y position

do_print_screen = false;
screen_count = 1;
set(gcf, 'Color', 'white');  % Make the figure's background transparent
set(gca, 'Color', 'white');  % Make the axes background transparent
% Animation loop
if animate
    for i = 1:length(t)

        if do_BRT
            
            delete(background_V);

            for v = 1:length(h)
                delete(V_0{v});
                if v == 1
                    do_background = true;
                    [background_V, V_0{v}] = doslice( V_small, h{v}.rel_states(i,:), h{v}.g, ...
                        x_center, y_center, do_background);
                else
                    do_background = false;
                    [~, V_0{v}] = doslice( V_big, h{v}.rel_states(i,:), h{v}.g, ...
                        x_center, y_center, do_background);
                end
            end
        end

        % Update vehicle positions
        [ego_shape, ego_wheel_shape] = update_vehicle_plot(x(i), y(i), theta(i), D1, D2, delta(i), ...
            wheel_l, wheel_w, ego_shape, ego_wheel_shape, 'b');

        for v = 1:length(h)
            [hdv_shape{v}, hdv_wheel_shape{v}] = update_vehicle_plot(x2{v}(i), y2{v}(i), theta2{v}(i), ...
                D1_h, D2_h, delta2{v}(i), wheel_l, wheel_w, hdv_shape{v}, hdv_wheel_shape{v}, 'r');
        end

        % Center the plot around the first vehicle
        x_center = x(i); % Ego vehicle's x position
        y_center = y(i); % Ego vehicle's y position
        range_x = 15; % Increased x-axis range for better visualization

        traj_ego_x = [traj_ego_x, x(i)];  % Add new ego position
        traj_ego_y = [traj_ego_y, y(i)];  % Add new ego position
    
        traj_human1_x = [traj_human1_x, x2{1}(i)];  % Add new human position
        traj_human1_y = [traj_human1_y, y2{1}(i)];  % Add new human position
        traj_human2_x = [traj_human2_x, x2{2}(i)];  % Add new human position
        traj_human2_y = [traj_human2_y, y2{2}(i)];  % Add new human position
    
    
        % Update the trajectory plot for ego
        set(traj_ego, 'XData', traj_ego_x, 'YData', traj_ego_y);
    
        % Update the trajectory plot for human
        set(traj_human1, 'XData', traj_human1_x, 'YData', traj_human1_y);
        set(traj_human2, 'XData', traj_human2_x, 'YData', traj_human2_y);


        if do_BRT
            % Update axis limits to keep ego vehicle centered on the x-axis

            xlim_inf = zeros(1,length(h));
            xlim_sup = zeros(1,length(h));
            ylim_inf = zeros(1,length(h));
            ylim_sup = zeros(1,length(h));

            for v = 1:length(h)
                xlim_inf(v) = min(V_0{v}.XData,[],"all");
                xlim_sup(v) = max(V_0{v}.XData,[],"all");
                ylim_inf(v) = min(V_0{v}.YData,[],"all");
                ylim_sup(v) = max(V_0{v}.YData,[],"all");

            end
            xlim([max(xlim_inf), min(xlim_sup)]);
            ylim([max(ylim_inf), min(ylim_sup)]);

        else
            % Update axis limits to keep ego vehicle centered on the x-axis
            xlim([x_center - range_x, x_center + range_x]);
            ylim([-y_lim, y_lim]);
        end

        if dovideo
            % Capture the current frame
            frame = getframe(gcf);

            % Write the frame to the video file
            writeVideo(writerObj, frame);
        end

        if do_print_screen &&  (mod(i, round((length(t)-1)/3)) == 0) && isempty(manual_time) || (i == 1) && isempty(manual_time)
        
            str = [save_str, num2str(screen_count),'.eps'];
            print('-depsc','-vector', '-r1200', str);
            screen_count = screen_count+1;
            
        elseif do_print_screen && i==manual_time(screen_count)
            str = [save_str, num2str(screen_count),'.eps'];
            print('-depsc','-vector', '-r1200', str);
            screen_count = screen_count+1;
        end

        if screen_count>4
            break
        end

        % Pause to create animation effect
        pause(anim_rate);

        % Redraw
        drawnow;
    end
end

% Plot setup
figure;
axis equal;
hold on;
grid on;
xlabel('$X\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$Y\ [m]$', 'Interpreter', 'latex', 'FontSize', 14)
set(gcf, 'Color', 'white');
set(gca, 'Color', 'white');

% Plot road boundaries
hold on;
yline(Yboundleft, 'k', 'LineWidth', 1);
yline(Yboundright, 'k', 'LineWidth', 1);

% Determine speed range for alpha normalization
v_min = model.v_min;
v_max = model.v_max;

% Frequency of rectangle plotting
num_steps = 10;

step_size = floor((length(t)-1)/num_steps);  % At most 20 rectangles



colour_hdv = ['r', 'y'];
% Plot HDV rectangles
for vi = 1:length(h)
    for i = 1:step_size:length(t)
        norm_alpha = (v2{vi}(i) - v_min) / (v_max - v_min + 1e-6);
        face_alpha = 0.2 + 0.6 * norm_alpha;
        plot_vehicle_rectangle(x2{vi}(i), y2{vi}(i), theta2{vi}(i), D1_h, D2_h, colour_hdv(vi), face_alpha, 0);
    end
end

% Plot ego trajectory rectangles
for i = 1:step_size:length(t)
    norm_alpha = (v(i) - v_min) / (v_max - v_min + 1e-6);  % Avoid zero denominator
    face_alpha = 0.2 + 0.6 * norm_alpha;  % Range from 0.2 to 0.8
    plot_vehicle_rectangle(x(i), y(i), theta(i), D1, D2, 'b', face_alpha, delta(i));
end

% Adjust figure limits to fit only vehicle trajectories
margin = 3; % Add a small margin

all_x = [x(1:i); vertcat(x2{:})];
if theta2{2}(1) == pi
    xlim([min(x)-margin, max(all_x)+margin]);
else
    xlim([min(all_x)-margin, max(x)+margin]);
end
ylim([-5 5]);


% Stretch the y-axis by setting the DataAspectRatio
dar = daspect; % Get current data aspect ratio
daspect([dar(1), dar(2) / 2, dar(3)]); % Halve the y-axis aspect ratio


end


% ------------------------------------------------------------------------%
% ------------------------------------------------------------------------%
% ------------------------------------------------------------------------%
%-------------------------------------------------------------------------%

% Function to update vehicle plot with rectangle and wheel
function [car_shape, wheel_shape] = update_vehicle_plot(x, y, theta, vehicle_length, vehicle_width, ...
    steering_angle, wheel_length, wheel_width, car_shape, wheel_shape, colour)

% Calculate rectangle corners
R = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % Rotation matrix based on heading
half_length = vehicle_length / 2;
half_width = vehicle_width / 2;

% Define the four corners of the vehicle rectangle
corners = [-half_length, -half_width;
    half_length, -half_width;
    half_length, half_width;
    -half_length, half_width]';
rotated_corners = R * corners; % Rotate the corners based on theta

% Translate the corners to the vehicle's position
x_corners = x + rotated_corners(1, :);
y_corners = y + rotated_corners(2, :);

if ~isempty(car_shape)
    delete(car_shape);
end
if ~isempty(wheel_shape)
    delete(wheel_shape);
end

% Plot vehicle body
car_shape = fill(x_corners, y_corners, colour); % Vehicle body

% Calculate front wheel position and orientation
front_center = [x; y] + R * [half_length; 0];
wheel_rotation = [cos(steering_angle), -sin(steering_angle); sin(steering_angle), cos(steering_angle)];
wheel_corners = [-wheel_length / 2, -wheel_width / 2;
    wheel_length / 2, -wheel_width / 2;
    wheel_length / 2, wheel_width / 2;
    -wheel_length / 2, wheel_width / 2]';
rotated_wheel = wheel_rotation * wheel_corners;

% Translate wheel corners to the front center of the vehicle
wheel_x = front_center(1) + rotated_wheel(1, :);
wheel_y = front_center(2) + rotated_wheel(2, :);

% Plot the front wheel
wheel_shape = fill(wheel_x, wheel_y, 'k');
end

%-------------------------------------------------------------------------%
% ------------------------------------------------------------------------%

function [background_V, slice0] = doslice( V, rel_states, g, x_c, y_c, do_background)


[~, index3] = min(abs(wrapToPi( rel_states(3) - g.vs{3}+eps)));
[~, index4] = min(abs( rel_states(4) - g.vs{4}));
[~, index5] = min(abs( rel_states(5) - g.vs{5}));
[~, index6] = min(abs( rel_states(6) -  g.vs{5}));

V_cont = squeeze(V(:,:,:,index4,index5,index6));

[X, Y] = meshgrid(x_c+g.vs{1}, y_c+g.vs{2});

if do_background
    [~, background_V] = contourf(X, Y, V_cont(:, :, index3)');

colormap;
colorbar;
else
    background_V = [];
end

hold on

% Zero level contour of V
[~, slice0] = contour(X, Y, V_cont(:, :, index3)', [0 0], 'LineWidth',1.5,'EdgeColor','red');



end

function plot_vehicle_rectangle(x, y, theta, vehicle_length, vehicle_width, colour, alpha, steering_angle)

R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
half_length = vehicle_length / 2;
half_width = vehicle_width / 2;

corners = [-half_length, -half_width;
            half_length, -half_width;
            half_length,  half_width;
           -half_length,  half_width]';
rotated_corners = R * corners;
x_corners = x + rotated_corners(1, :);
y_corners = y + rotated_corners(2, :);

fill(x_corners, y_corners, colour, 'FaceAlpha', alpha, 'EdgeColor', colour);

% Wheel dimensions
wheel_length = 1;         
wheel_width = 0.2;

front_center = [x; y] + R * [half_length; 0];
wheel_rotation = [cos(steering_angle), -sin(steering_angle);
                  sin(steering_angle),  cos(steering_angle)];
wheel_corners = [-wheel_length / 2, -wheel_width / 2;
                  wheel_length / 2, -wheel_width / 2;
                  wheel_length / 2,  wheel_width / 2;
                 -wheel_length / 2,  wheel_width / 2]';
rotated_wheel = wheel_rotation * wheel_corners;
wheel_x = front_center(1) + rotated_wheel(1, :);
wheel_y = front_center(2) + rotated_wheel(2, :);

fill(wheel_x, wheel_y, 'k', 'FaceAlpha', alpha, 'EdgeColor', 'k', 'LineWidth', 0.3);

end
