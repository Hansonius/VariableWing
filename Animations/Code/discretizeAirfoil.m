function discretized = discretizeAirfoil(data, num_disc)
    [~, col] = size(data);
    
    
    
    % LE and TE found as where the min and max x locations are
    % LE INDEX ASSUMED TO BE 1
    [~, TE_index] = max(data(:, 1));
    
    LE = data(1, :);
    TE = data(TE_index, :);
    
    % In the case that there is some weird angle to the airfoil geometry,
    % we need to capture this when breaking up the geometry
    theta = atan((TE(2) - LE(2)) / (TE(1) - LE(1)));
    
    
    if col == 2
        rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    elseif col == 3
        rotation_matrix = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    else
        disp('Cannot read matrix properly. Too many columns')
    end
    
    % Rotate about the LE, and then add the LE back in
    rotated_points = (rotation_matrix * (data - LE)')' + LE;
   
    
    % Cannot just check the y position of each point to split as there can be portions above the chord line on
    % the underside. Really we need everything before and after the TE
    before_TE = rotated_points(2:TE_index - 1, :);
    after_TE = rotated_points(TE_index + 1:end, :);
    
    
    
    
    
    % Add in the LE and TE to each of these groups to help with the
    % discretization
    % Ensure to redefine the TE first if it got rotated
    TE = rotated_points(TE_index, :);
    before_TE = [LE; before_TE; TE];
    after_TE = [LE; after_TE; TE];
    
    
    % In the case that for some reason these points are out of order sort
    % them by their x-values
    before_TE = sortrows(before_TE);
    after_TE = sortrows(after_TE);
    
    
    % Check if the number of discretizations is even or not
    if mod(num_disc, 2) ~= 0
        disp('Made number of discretized points even by adding 1');
        num_disc = num_disc + 1;
    end
    
    x_spacing = linspace(LE(1), TE(1), num_disc / 2 + 1);

    
    % Remove the LE and TE from the x-spacing
    x_spacing(1) = [];
    x_spacing(end) = [];
    
    before_disc = zeros(length(x_spacing), col);
    after_disc = zeros(length(x_spacing), col);
    
    % Build up the above spacing
    for i = 1:length(x_spacing)
        right_index = find(before_TE(:, 1) > x_spacing(i), 1);
        
        left_x = before_TE(right_index - 1, 1);
        right_x = before_TE(right_index, 1);
        
        weight = (x_spacing(i) - left_x) / (right_x - left_x);
        
        before_disc(i, :) = before_TE(right_index - 1, :) + weight * (before_TE(right_index, :) - before_TE(right_index - 1, :));
    end
    
    % Build up the below spacing
    for i = 1:length(x_spacing)
        right_index = find(after_TE(:, 1) > x_spacing(i), 1);
        
        left_x = after_TE(right_index - 1, 1);
        right_x = after_TE(right_index, 1);
        
        weight = (x_spacing(i) - left_x) / (right_x - left_x);
        
        after_disc(i, :) = after_TE(right_index - 1, :) + weight * (after_TE(right_index, :) - after_TE(right_index - 1, :));
    end
    
    % Put in everything in order
    rotated_discretized = [LE; before_disc; TE; flip(after_disc, 1)];
    
    discretized = (rotation_matrix' * (rotated_discretized - LE)')' + LE;
end
    
    
        
        
    
    
    
    
    
    
    
    
    
    
    
    