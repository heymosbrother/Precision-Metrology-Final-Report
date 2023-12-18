x = [0.1; 0.2];
[x_m, y_m] = gpsdata(x(1),x(2));

function [x_m, y_m] = gpsdata(x_t, y_t)
    % 單位m
    x_m = x_t + random('Normal', 0, 5);
    y_m = y_t + random('Normal', 0, 5);
end