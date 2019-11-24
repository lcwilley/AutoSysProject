function r_ellipse = error_ellipse(center,covariance)
    % Given the center of an ellipse and the covariance, returns the values
    % of the corresponding ellipse

    % Calculate the eigenvectors and eigenvalues
    [vecs, vals] = eig(covariance);

    % Find the largest and smallest eigenvalues, as well as the largest
    % eigenvector
    [max_val,max_val_ind] = max(diag(vals));
    min_val = min(diag(vals));
    max_vec = vecs(:,max_val_ind);
    
    % Calculate the angle between the x-axis and the largest eigenvector
    phi = atan2(max_vec(2), max_vec(1));
    
    % Get the 95% confidence interval error ellipse
    chisquare_val = 2.4477;
    th = linspace(0,2*pi,50);
    a = chisquare_val * sqrt(max_val);
    b = chisquare_val * sqrt(min_val);

    % Get ellipse X and Y coordinates
    ellipse_x_r  = a*cos(th);
    ellipse_y_r  = b*sin(th);

    % Rotate the ellipse
    R = [cos(phi) sin(phi); -sin(phi) cos(phi)];
    r_ellipse = R * [ellipse_x_r;ellipse_y_r] + center;

end
