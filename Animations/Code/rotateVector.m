function rotated_vec = rotateVector(vec, axis, theta)
    % Get the unit axis
    mag = norm(axis, 2);
    k = axis ./ mag;
    
    k_cross_v = cross(k, vec);
    k_dot_v = dot(k, vec);
    
    rotated_vec = vec .* cos(theta) + k_cross_v .* sin(theta) + k .* k_dot_v * (1 - cos(theta));
end
    