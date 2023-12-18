function [data_k, K, previous, previous_esterr] = positionfusion(data_t, K, previous, previous_esterr)
    % kalman filter
    dt = 0.01; % 採樣頻率 s
    a = acceldata(14^2/1000);
    a_x = a(1);
    a_y = a(2);
    v_x = cumtrapz(dt, a_x);
    v_y = cumtrapz(dt, a_y);
    p = gpsdata(data_t(2), data_t(3));
    p_x = p(1);
    p_y = p(2);
    measure = [data_t(1); p_x; v_x; a_x; p_y; v_y; a_y];
    transition = [1 0 0 0 0 0 0;
                  0 dt 0.5*dt^2 0 0 0;
                  0 0 1 dt 0 0 0;
                  0 0 0 1 0 0 0;
                  0 0 0 0 1 dt 0.5*dt^2;
                  0 0 0 0 0 1 dt;
                  0 0 0 0 0 0 1 ];
    data_e = transition*measure;
%     initial = [1000; 14; 0; 0; 0; 14^2/1000];
    initial_esterr = [1; 1; 1; 1; 1; 1; 1];
    uncertainty = [0; (5/0.0987)^2; (5/0.0987)^2; 0; 0; (19.6/0.0987)^2; (19.6/0.0987)^2];

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
    if data_t(1) == 0
        K = initial_esterr / (initial_esterr + uncertainty);
    else
        K = previous_esterr / (previous_esterr + uncertainty);
    end

    % 更新估計值和估計誤差
    if data_t(1) == 0
        data_k = data_e;
        previous = data_k;
        previous_esterr = (1 - K) * initial_esterr;
    else
        data_k = previous + K * (data_e - previous);
        previous = data_k;
        previous_esterr = (1 - K) * previous_esterr;
    end

end