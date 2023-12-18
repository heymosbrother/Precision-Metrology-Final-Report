function [x_m, y_m] = gpsdata(x_t, y_t)
    % 單位m
    x_m = x_t + random('Normal', 0, 5/0.0987);
    y_m = y_t + random('Normal', 0, 5/0.0987);
end

function [xdd_m, ydd_m] = acceldata(a)
    % 單位m/s^2 a=v^2/r
    xdd_m = random('Normal', 0, 19.6/0.0987);
    ydd_m = a + random('Normal', 0, 19.6/0.0987);
end