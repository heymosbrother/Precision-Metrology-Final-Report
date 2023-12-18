function output = gpsdata(input)
    % 單位m
    output(1) = input(1) + random('Normal', 0, 5);
    output(2) = input(2) + random('Normal', 0, 5);
end

function output = acceldata(a)
    % 單位m/s^2 a=v^2/r
    output(1) = random('Normal', 0, 19.6);
    output(2) = a + random('Normal', 0, 19.6);
end

function output = gyrodata(input)
    % 單位deg/s
    output = 14/1000 + random('Normal', 0, 125);
end

function output = magdata(input)
    % 單位uT 初始(?)
    output = input + random('Normal', 0, 0.75);
end