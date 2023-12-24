x = [0.1 0.2];
p = gpsdata(x);

function output = gpsdata(input)
    % 單位m
    output(1) = input(1) + random('Normal', 0, 5);
    output(2) = input(2) + random('Normal', 0, 5);
end