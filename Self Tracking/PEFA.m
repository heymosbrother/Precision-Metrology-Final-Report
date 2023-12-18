ld = load('CircularTrajectorySensorData.mat');

Fs = ld.Fs; % maximum MARG rate (Magnetic，Angular and Gravity) 定義最大採樣率，保持數據同步
gpsFs = ld.gpsFs; % maximum GPS rate
ratio = Fs./gpsFs;
refloc = ld.refloc; %參考位置

trajOrient = ld.trajData.Orientation;
trajVel = ld.trajData.Velocity;
trajPos = ld.trajData.Position;
trajAcc = ld.trajData.Acceleration;
trajAngVel = ld.trajData.AngularVelocity; %軌跡資料，方向、速度、位置、加速度、角加速度

accel = ld.accel;
gyro = ld.gyro;
mag = ld.mag;
lla = ld.lla; %海拔
gpsvel = ld.gpsvel; %感測器資料

fusionfilt = insfilterAsync('ReferenceLocation', refloc); %異步濾波器

Nav = 100;
initstate = zeros(28,1);
initstate(1:4) = compact( meanrot(trajOrient(1:Nav)));
initstate(5:7) = mean( trajAngVel(10:Nav,:), 1);
initstate(8:10) = mean( trajPos(1:Nav,:), 1);
initstate(11:13) = mean( trajVel(1:Nav,:), 1);
initstate(14:16) = mean( trajAcc(1:Nav,:), 1);
initstate(23:25) = ld.magField;

% The gyroscope bias initial value estimate is low for the Z-axis. This is
% done to illustrate the effects of fusing the magnetometer in the
% simulation.
initstate(20:22) = deg2rad([3.125 3.125 3.125]);
fusionfilt.State = initstate;

fusionfilt.QuaternionNoise = 1e-2;
fusionfilt.AngularVelocityNoise = 100;
fusionfilt.AccelerationNoise = 100;
fusionfilt.MagnetometerBiasNoise = 1e-7;
fusionfilt.AccelerometerBiasNoise = 1e-7;
fusionfilt.GyroscopeBiasNoise = 1e-7;       % 設定process noise

Rmag = 0.4;
Rvel = 0.01;
Racc = 610;
Rgyro = 0.76e-5;
Rpos = 3.4;     % 共變異數(variation，for normal distribution)

fusionfilt.StateCovariance = diag(1e-3*ones(28,1));

useErrScope = true; % Turn on the streaming error plot.
usePoseView = true; % Turn on the 3D pose viewer.
if usePoseView
    posescope = PoseViewerWithSwitches(...
        'XPositionLimits', [-1010 1010], ...
        'YPositionLimits', [-1010, 1010], ...
        'ZPositionLimits', [-10 10]);
end
f = gcf;

% 計算error
if useErrScope
    errscope = HelperScrollingPlotter(...
        'NumInputs', 4, ...
        'TimeSpan', 10, ...
        'SampleRate', Fs, ...
        'YLabel', {'degrees', ...
        'meters', ...
        'meters', ...
        'meters'}, ...
        'Title', {'Quaternion Distance', ...
        'Position X Error', ...
        'Position Y Error', ...
        'Position Z Error'}, ...
        'YLimits', ...
        [ -1, 30
        -2, 2
        -2 2
        -2 2]);
end

maxPosError = 0;

for ii=1:size(accel,1)
    fusionfilt.predict(1./Fs);

    % Fuse Accelerometer
    if (f.UserData.Accelerometer) && ...
        mod(ii, fix(Fs/f.UserData.AccelerometerSampleRate)) == 0

        fusionfilt.fuseaccel(accel(ii,:), Racc);
    end

    % Fuse Gyroscope
    if (f.UserData.Gyroscope) && ...
        mod(ii, fix(Fs/f.UserData.GyroscopeSampleRate)) == 0

        fusionfilt.fusegyro(gyro(ii,:), Rgyro);
    end

    % Fuse Magnetometer
    if (f.UserData.Magnetometer) && ...
        mod(ii, fix(Fs/f.UserData.MagnetometerSampleRate)) == 0

        fusionfilt.fusemag(mag(ii,:), Rmag);
    end

    % Fuse GPS
    if (f.UserData.GPS) && mod(ii, fix(Fs/f.UserData.GPSSampleRate)) == 0
        fusionfilt.fusegps(lla(ii,:), Rpos, gpsvel(ii,:), Rvel);
    end

    % 計算當前時刻的位置誤差
    posErr = p - trajPos(ii,:);

    % 更新最大位置誤差
    maxPosError = max(maxPosError, norm(posErr));

    % Plot the pose error (圖像結果)
    [p,q] = pose(fusionfilt);
    posescope(p, q, trajPos(ii,:), trajOrient(ii));

    orientErr = rad2deg(dist(q, trajOrient(ii) ));
    posErr = p - trajPos(ii,:);
    errscope(orientErr, posErr(1), posErr(2), posErr(3));
end