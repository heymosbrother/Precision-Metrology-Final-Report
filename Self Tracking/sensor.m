function output = gpsdata(input)
    % 單位m
    x_m = input(1) + random('Normal', 0, 5);
    y_m = input(2) + random('Normal', 0, 5);
    output = [x_m y_m];
end

function output = acceldata(a)
    % 單位m/s^2 a=v^2/r
    x_m = random('Normal', 0, 19.6);
    y_m = a + random('Normal', 0, 19.6);
    output = [x_m y_m];
end

function output = gyrodata(input)
    % 單位deg/s
    output = 14/1000 + random('Normal', 0, 125);
end

function output = magdata(input)
    % 單位uT 初始(?)
    output = input + random('Normal', 0, 0.75);
end