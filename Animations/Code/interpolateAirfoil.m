function interpolated_airfoil = interpolateAirfoil(airfoil_1, airfoil_2, weight, num_disc)
    % Ensure that the airfoils are discretized to have the same number of
    % points
    airfoil_1 = discretizeAirfoil(airfoil_1, num_disc);
    airfoil_2 = discretizeAirfoil(airfoil_2, num_disc);
    
    diff = airfoil_2 - airfoil_1;
    
    interpolated_airfoil = airfoil_1 + weight * diff;
end