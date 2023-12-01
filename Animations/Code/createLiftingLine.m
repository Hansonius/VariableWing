function [dat_file_points, connectivity_mat] = createLiftingLine(point_data_file, airfoils, airfoil_change_indeces, line_count_3D)
    point_data = readmatrix(point_data_file);
    
    QC_pos = point_data(:, 1:3);
    
    chordwise_untwisted_vecs = point_data(:, 4:6);
    normal_untwisted_vecs = point_data(:, 7:9);
    twist_angles = point_data(:, 10);
    chord_lengths = point_data(:, 11);
    
    spanwise_vecs = zeros(size(chordwise_untwisted_vecs));
    twisted_chordwise_vecs = zeros(size(chordwise_untwisted_vecs));
    twisted_normal_vecs = zeros(size(normal_untwisted_vecs));
    
    LE_pos = zeros(size(QC_pos));
    TE_pos = zeros(size(QC_pos));
    
    [rows, ~] = size(point_data);
    
    for i = 1:rows
        % Get the spanwise vectors as the cross product between the
        % chordwise and normal vectors
        spanwise_vecs(i, :) = cross(chordwise_untwisted_vecs(i, :), normal_untwisted_vecs(i, :));
        
        % Twist the vectors. Really only care about the chordwise ones
        twisted_chordwise_vecs(i, :) = rotateVector(chordwise_untwisted_vecs(i, :), spanwise_vecs(i, :), twist_angles(i));
        twisted_normal_vecs(i, :) = rotateVector(normal_untwisted_vecs(i, :), spanwise_vecs(i, :), twist_angles(i));
        
        LE_pos(i, :) = QC_pos(i, :) + 0.25 * chord_lengths(i) * twisted_chordwise_vecs(i, :);
        TE_pos(i, :) = QC_pos(i, :) - 0.75 * chord_lengths(i) * twisted_chordwise_vecs(i, :);
    end
    
    
    % First, find the chord lengths and critical angles
    thetas = zeros(rows, 1);
    gammas = zeros(rows, 1);
    psis = zeros(rows, 1);

    for i = 1:rows
        % Rotation about X axis
        thetas(i) = atan((LE_pos(i, 3) - TE_pos(i, 3)) / (LE_pos(i, 2) - TE_pos(i, 2)));

        % Rotation about Y axis
        gammas(i) = atan(-normal_untwisted_vecs(i, 1) / normal_untwisted_vecs(i, 3));
        
        % Rotation about Z axis
        psis(i) = atan((LE_pos(i, 1) - TE_pos(i, 1)) / (LE_pos(i, 2) - TE_pos(i, 2)));
    end
    
    
    if ~exist('line_count_3D')
        line_count_3D = 100;
    end
    
    
    
    
   
    
    
    airfoil_data = cell(length(airfoils), 1);
    for i = 1:length(airfoils)
        airfoil_data{i} = readmatrix(airfoils(i));
    end
    
    
    discretized_airfoils = cell(rows, 1);
    current_airfoil_index = 1;
 
    
    
    for i = 1:rows
        % First find what airfoil is being used at the airstation
        
        % Do not want to go beyond the bounds of the indeces
        if current_airfoil_index <= length(airfoil_change_indeces)
            % If we come across a point where the airfoil must change from the
            % index, then advance it
            if i == airfoil_change_indeces(current_airfoil_index)
                current_airfoil_index = current_airfoil_index + 1;
            end
        end
        
        % If the current airfoil is the last one in the series, then we
        % don't interpolate
        if (current_airfoil_index ~= length(airfoils)) && (current_airfoil_index ~= 1)
            % Weighting. A weight of 1 corresponds to fully using the next airfoil in the series, while a weight of 0 means just using the first airfoilairf 
            weight = 1 - (airfoil_change_indeces(current_airfoil_index) - i) / (airfoil_change_indeces(current_airfoil_index) - airfoil_change_indeces(current_airfoil_index - 1));
            discretized_airfoils{i} = interpolateAirfoil(airfoil_data{current_airfoil_index}, airfoil_data{current_airfoil_index + 1}, weight, line_count_3D);
            
        elseif (current_airfoil_index ~= length(airfoils))
            weight = 1 - (airfoil_change_indeces(current_airfoil_index) - i) / (airfoil_change_indeces(current_airfoil_index) - 1);
            discretized_airfoils{i} = interpolateAirfoil(airfoil_data{current_airfoil_index}, airfoil_data{current_airfoil_index + 1}, weight, line_count_3D);
            
        else
            discretized_airfoils{i} = discretizeAirfoil(airfoil_data{current_airfoil_index}, line_count_3D);
        end
        
        
        %discretized_airfoils{i} = discretizeAirfoil(airfoil_data{current_airfoil_index}, line_count_3D);
        
        discretized_airfoils{i} = [zeros(line_count_3D, 1), -1 .* discretized_airfoils{i}(:, 1), discretized_airfoils{i}(:, 2)];
        airfoil = discretized_airfoils{i};
        
        discretized_airfoils{i} = discretized_airfoils{i} .* chord_lengths(i);
        
        
        % Then rotate them into correct position
        % Theta is about positive x-axis
        theta_rot = [1,     0,                 0;
                     0, cos(thetas(i)), -sin(thetas(i));
                     0, sin(thetas(i)), cos(thetas(i))];
        
        % Gamma is about negative y direction
        gamma_rot = [cos(gammas(i)),  0,  -sin(gammas(i));
                           0,         1,         0;
                     sin(gammas(i)),  0,  cos(gammas(i))];
        
        % Negative rotation as sweep goes about the z-axis in the negative direction (i.e. positive sweep means CW angle) 
        psi_rot = [cos(psis(i)), sin(psis(i)), 0;
                   -sin(psis(i)), cos(psis(i)),  0;
                        0,            0,        1];
        
                    
        for j = 1:line_count_3D
            discretized_airfoils{i}(j, :) = psi_rot * theta_rot * gamma_rot * (discretized_airfoils{i}(j, :))';
        end
        
        
        
        
        % Now find the offset to match up the leading edges
        offset = [LE_pos(i, 1) - airfoil(1, 1), LE_pos(i, 2) - airfoil(1, 2), LE_pos(i, 3) - airfoil(1, 3)];

        
        discretized_airfoils{i}(:, 1) = discretized_airfoils{i}(:, 1) + offset(1);
        discretized_airfoils{i}(:, 2) = discretized_airfoils{i}(:, 2) + offset(2);
        discretized_airfoils{i}(:, 3) = discretized_airfoils{i}(:, 3) + offset(3);
        
    end
    
    
    
    % Points for use in data file
    point_labels = (1:(rows * line_count_3D))';
    point_locations = zeros(rows * line_count_3D, 3);
    dat_file_points = [point_labels, point_locations];
    
    for i = 1:rows
        dat_file_points(((i-1)*line_count_3D + 1):i*line_count_3D, 2:4) = discretized_airfoils{i};
    end
    
    % Generate the elements for the .dat file
    num_elements = (rows - 1) * line_count_3D;
    connectivity_mat = zeros(num_elements, 4);
    
    % Build up the elements
    count = 1;
    for i = 1:(rows - 1)
        for j = 1:line_count_3D
            if j ~= line_count_3D
                point_A_ID = (i - 1) * line_count_3D + j;
                point_B_ID = point_A_ID + 1; % loop around
                point_C_ID = i * line_count_3D + j + 1;
                point_D_ID = point_C_ID - 1;
            
            % On the last element, we've 'looped around' the airfoil so
            % that points B and D are the at the first index of each
            % airfoil
            else
                point_A_ID = i * j;
                point_B_ID = (i - 1) * j + 1;
                point_C_ID = i * j + 1;
                point_D_ID = (i + 1) * j;
            end
            
            connectivity_mat(count, :) = [point_A_ID, point_B_ID, point_C_ID, point_D_ID];
            count = count + 1;
        end
    end
end

    
    