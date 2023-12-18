

function [data_k] = positionfusion(data_t)
    % kalman filter
    dt = 0.5; % 採樣頻率 s
    a = acceldata(14^2/1000);
    a_x = a(1);
    a_y = a(2);
    v_x = cumtrapz(dt, a_x);
    v_y = cumtrapz(dt, a_y);
    p = gpsdata(data_t(2), data_t(3));
    p_x = p(1);
    p_y = p(2);
    measure = [p_x; v_x; a_x; p_y; v_y; a_y];
    transition = [1 dt 0.5*dt^2 0 0 0;
                  0 1 dt 0 0 0;
                  0 0 1 0 0 0;
                  0 0 0 1 dt 0.5*dt^2;
                  0 0 0 0 1 dt;
                  0 0 0 0 0 1 ];
    data_e = transition*measure;
%     initial = [1000; 14; 0; 0; 0; 14^2/1000];
    initial_esterr = [1; 1; 1; 1; 1; 1];
    uncertainty = [(5/0.0987)^2; (5/0.0987)^2; 0; 0; (19.6/0.0987)^2; (19.6/0.0987)^2];

%     % Karman Gain
%     if K == 0
%         K = initial_esterr/(initial_esterr + uncertainty);
%         data_k = initial + K*(data_e - initial);
%         previous = data_k;
%         previous_esterr = (1-K)*initial_esterr;
%     else
%         K = previous_esterr/(previous_esterr + uncertainty);
%         data_k  = previous + K*(data_e - previous);
%         previous = data_k;
%         previous_esterr = (1-K)*previous_esterr;
%     end
    
    % Kalman Gain
    if isempty(K)
        K = initial_esterr / (initial_esterr + uncertainty);
    else
        K = previous_esterr / (previous_esterr + uncertainty);
    end

    % 更新估計值和估計誤差
    if isempty(previous)
        data_k = transition * measure;
        previous = data_k;
        previous_esterr = (1 - K) * initial_esterr;
    else
        data_k = previous + K * (measure - previous);
        previous = data_k;
        previous_esterr = (1 - K) * previous_esterr;
    end

end