function generateDatFile(point_data, connectivity_data, file_location, filename)
    file_path = char(file_location + filename);
    file_ID = fopen(file_path, 'w');
    
    [num_points, ~] = size(point_data);
    
    spaces = ",";
    
    % Put in the GRID points to denote the point data 
    for i = 1:num_points
        data_type = "GRID";
        point_ID = num2str(point_data(i, 1));
        coordinate_system = "0";
        X_location = num2str(point_data(i, 2));
        Y_location = num2str(point_data(i, 3));
        Z_location = num2str(point_data(i, 4));
        CD = "";
        PS = "";
        SEID = "0";
        
        line = char(data_type + spaces + point_ID + spaces + coordinate_system + spaces + X_location + spaces + Y_location + spaces + Z_location + spaces + CD + spaces + PS + spaces + SEID + "\n");
        
        fprintf(file_ID, line);
    end
    
    % Put in the quad data to connect the points
    [num_elements, ~] = size(connectivity_data);
    
    for i = 1:num_elements
        data_type = "CQUAD4";
        element_ID = num2str(i);
        PID = "1";
        point_A_ID = connectivity_data(i, 1);
        point_B_ID = connectivity_data(i, 2);
        point_C_ID = connectivity_data(i, 3);
        point_D_ID = connectivity_data(i, 4);
        theta = "";
        MCID = "";
        
        line = char(data_type + spaces + element_ID + spaces + PID + spaces + point_A_ID + spaces + point_B_ID + spaces + point_C_ID + spaces + point_D_ID + spaces + theta + spaces + MCID + "\n");
        
        fprintf(file_ID, line);
    end
    
    fclose(file_ID);

end