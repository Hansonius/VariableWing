%% Animate variable wing
clear all
clc

sd_file_path = "C:\Users\Josh\OneDrive - The University of Texas at Austin\Desktop\School\Fall 2023\Clarke Research\Clarke_Fa23\Data\";
sd_file_name = "variable_wing_data_slight_underdamped.txt";

sweep_data_file = sd_file_path + sd_file_name;

data = readmatrix(sweep_data_file);
time = data(:, 1);
as_count = data(:, 2);
phi_z = data(:, 5);


%% Creating the lifting line
% Data file of placements of the different airstations in the lifting line
% frame as output by the GenerateLiftingLines() function
%point_data_file = 'H:\Hermes_Scratch\source\GeometryTesting\GeometryTesting\Knight_Hefner_10AS.txt';
point_data_file_path = "C:\Users\Josh\OneDrive - The University of Texas at Austin\Desktop\School\Fall 2023\Clarke Research\Clarke_Fa23\LiftingLines\";

file_name = "Knight_Hefner_4AS.txt";

point_data_file = point_data_file_path + file_name;

% Sequence of airfoil geometries along the lifting line from innermost to
% outer most. Recommended inputs are .txt and .dat, but anything compatible
% with the readmatrix() function should work
path = "C:\Users\Josh\OneDrive - The University of Texas at Austin\Desktop\School\Fall 2023\Clarke Research\Airfoils\";
%airfoils = path + ["NACA0012.txt", "NACA63412.txt", "NASA_SC2_0612.txt", "USA5Airfoil.txt"];
airfoils = path + ["NACA63412.txt", "NACA0012.txt"];

% Indeces of where to change from one airfoil to the next. Should be of
% length number of airfoils - 1. For instance, if the first number in this
% list is 3, then the third airstation will switch to have the geometry of
% the second airfoil specified in the airfoils list
airfoil_change_indeces = [50];

% Optional Boolean variable asking whether you want the model to be made 3 
% dimensional. Deafult is set to false
%model_3D = true;

% Optional parameter for how many lines to draw for the 3 dimensional shape. Only
% utilized if model_3D is true. Deafult is set to 100
line_count_3D = 2;

% Plots the lifting line specified by these above parameters and also
% creates gives the parameters needed 
[dat_file_points, connectivity_mat] = createLiftingLine(point_data_file, airfoils, airfoil_change_indeces, line_count_3D);


%% Assigning airstations to the connectivity matrix
% We get the airstation index of a certain element of the lifiting line by
% using the line_count_3D. Each airstation will be comprised of
% line_count_3D number of elements. Recall that for every airstation there
% are an associated 4 points and there is also a set of points for the
% leading and trailing edges. This means that there are N+1 sections on the
% displayed lifting line as the airstations are what are being drawn.
section_count = size(connectivity_mat, 1) / line_count_3D;
num_as = section_count - 1;

% Vector to assign each row of the connectivity matrix what section of the
% lifting line they are in
section_num_vec = repelem(1:section_count, line_count_3D);


%% Plotting
lifting_line_root_qc_location = [-0.41667, -0.25, 0];

% Shifts all the points into position
dat_file_points(:, 2:4) = dat_file_points(:, 2:4) + lifting_line_root_qc_location;
rotation_point = [0, 0, 0];

% Create a dummy to use for every update in the loop
dummy_points = dat_file_points;

video_file = 'variable_wing.mp4';
video_obj = VideoWriter(video_file, 'MPEG-4');
open(video_obj);

%fig = figure('Visible', 'off');
fig = figure();

for i = 1:length(phi_z)
    for j = 1:size(dat_file_points, 1)
        dummy_points(j, 2:4) = rotateVector(dat_file_points(j, 2:4) - rotation_point, [0, 0, 1], phi_z(i));
    end

    for k = 1:size(connectivity_mat, 1)
        % Check if the airstation is active or not by looking if the
        % section number is greater than the section at which the
        % airstations are turned off

        % Green means that the airstation is active and red means that it
        % is not
        if section_num_vec(k) > (num_as - as_count(i))
            patch('Faces', [1 2 3 4], 'Vertices', dummy_points(connectivity_mat(k, :), 2:4), 'FaceColor', [0.7, 0.7, 0.7], 'EdgeColor', 'none');
        else
            patch('Faces', [1 2 3 4], 'Vertices', dummy_points(connectivity_mat(k, :), 2:4), 'FaceColor', 'r', 'EdgeColor', 'none');
        end
        hold on;
    end
    xlabel('x (m)')
    ylabel('y (m)')
    title('Variable Wing Sweep Visualization')
    view(0, 90)
    xlim([0, 2.2])
    ylim([-1.75, 0])
    % Capture the frame for the video
    frame_data = getframe(fig);

    % Write the frame to the video file
    writeVideo(video_obj, frame_data);

    % Clear the figure 
    clf;
end

% Close the figure
close(fig);

% Close the video
close(video_obj);
