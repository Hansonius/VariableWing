%% Writing input files
clear all
clc

%% Input Data
% Knight-Hefner data
% root_chord_length = 0.106 * acos(-1) * 2.5 / 5.0;
% root_offset = [5/12, 0, 0];
% tip_radius = 2.5;
% taper_ratio = 1;
% root_angle_offset = 0;
% total_twist = 0;
% sweep = 0;
% dihedral = 0;

% Root chord length
root_chord_length = 1;
% Placement of root quarter chord in lifting line frame
root_offset = [0, 0, 0];
% X coordinate of tip quarter chord
tip_radius = 7;
% Tip chord length / Root chord length
taper_ratio = 0.2;
% Twist of root chord about its chordwise and normal vectors
root_angle_offset = deg2rad(15);
% Total twist from root to tip. Typically is negative
total_twist = deg2rad(0);
% Angle between chord lines and the XZ plane of the lifting line frame
sweep = deg2rad(0.0);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SHOULD THE DIHEDRAL BE ALLOWED TO BE DEFINED WHEN MAKING THE LIFTING LINE
% OR IS IT SOMETHING THAT IS SOLELY APPLIED AT THE ROTOR LEVEL???

% Angle between normal vectors of airstations and the YZ plane of the
% lifting line frame
dihedral = 0.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


root_cw_vec = [sin(sweep), cos(sweep), 0];

root_norm_vec = [-sin(dihedral), 0, cos(dihedral)];
%root_norm_vec = [0, 0, 1];

tip_cw_vec = root_cw_vec;
tip_norm_vec = root_norm_vec;
num_airstations = 10;

file_location = "C:\Users\Josh\OneDrive - The University of Texas at Austin\Desktop\School\Fall 2023\Clarke Research\Clarke_Fa23\LiftingLines\";
filename = "basic_wing4.txt";

%% Generated Data
positions = zeros(num_airstations, 3);
chordwise_vecs = zeros(num_airstations, 3);
norm_vecs = zeros(num_airstations, 3);
twists = zeros(num_airstations, 1);
chord_lengths = zeros(num_airstations, 1);

% Total length of quarter chord line
root_offset_mag = norm(root_offset, 2);

lifting_surface_length = (tip_radius - root_offset_mag) / (cos(sweep) * cos(dihedral));
%lifting_surface_length = (tip_radius - root_offset_mag) / cos(sweep);

% Generate all relevant parameters for the airstations
for i = 1:num_airstations
    spanwise_loc = (lifting_surface_length / num_airstations) * ((i - 1) + 0.5);
    
    positions(i, :) = [spanwise_loc * cos(sweep) * cos(dihedral), spanwise_loc * -sin(sweep) * cos(dihedral), spanwise_loc * sin(dihedral)] + root_offset;
    %positions(i, :) = [spanwise_loc * cos(sweep), spanwise_loc * -sin(sweep), 0] + root_offset;
    
    chordwise_vecs(i, :) = [sin(sweep), cos(sweep), 0];
    
    norm_vecs(i, :) = [-sin(dihedral), 0, cos(dihedral)];
    %norm_vecs(i, :) = [0, 0, 1];
    
    twists(i) = root_angle_offset + total_twist * spanwise_loc / lifting_surface_length;
    chord_lengths(i) = root_chord_length * (1 - ( 1 - taper_ratio) * spanwise_loc / lifting_surface_length);
end

% Set the root chord data
root_chord_data = [root_offset, root_cw_vec, root_norm_vec, root_angle_offset, root_chord_length];

% Set the tip chord data
tip_pos = root_offset + [lifting_surface_length * cos(sweep) * cos(dihedral), lifting_surface_length * cos(dihedral) * -sin(sweep), lifting_surface_length * sin(dihedral)];
%tip_pos = root_offset + [lifting_surface_length * cos(sweep), lifting_surface_length * -sin(sweep), 0];

tip_chord_data = [tip_pos, tip_cw_vec, tip_norm_vec, root_angle_offset + total_twist, root_chord_length * taper_ratio];

% Create a master matrix of all data
master_data = [root_chord_data;
               positions, chordwise_vecs, norm_vecs, twists, chord_lengths;
               tip_chord_data];

%% Writing the data to a file
[rows, cols] = size(master_data);
file_path = char(file_location + filename);
file_ID = fopen(file_path, 'w');
fprintf(file_ID, '%i %i\n', rows, cols);
writematrix(master_data, file_path, 'Delimiter', ' ', 'WriteMode', 'append');
fclose(file_ID);

% Outputs the generated file contents in the command window
%type(file_path)
