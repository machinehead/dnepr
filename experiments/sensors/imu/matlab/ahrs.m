function [] = ahrs()
%AHRS Matlab port of Arduino AHRS by Pololu.

    % angle comparison plot data
    times = [];
    angDiffs = [];
    f1 = figure();

    % accelerometer sensivity: +/-2g = +/- 2048
    GRAVITY = 1024;
    
    % direction of axes
    % x - forward
    % y - right
    % z - down
    SENSOR_SIGN = [1 1 1 -1 -1 -1 1 1 1]; % gyro, accel, magnetometer
    GYRO_SIGN = SENSOR_SIGN(1:3); 
    ACC_SIGN = SENSOR_SIGN(4:6);
    MAG_SIGN = SENSOR_SIGN(7:9);

    % magnetometer calibration constants
    MAG_MIN = [-883 -942 -533];
    MAG_MAX = [320 481 452];
    
    % current angles relative to world coordinates, in degrees
    roll = 0;
    pitch = 0;
    yaw = 0;
    
    % current magnetometer heading
    MAG_Heading = 0;
    
    % L3G4200D gyro: 2000 dps full scale
    % 70 mdps/digit; 1 dps = 0.07
    GYRO_GAIN = 0.07;
    
    % gyro drift corrections
    Omega_P = [0 0 0]; % Omega Proportional correction
    Omega_I = [0 0 0]; % Omega Integrator
    
    % coeffs for roll & pitch correction
    Kp_ROLLPITCH = 0.02;
    Ki_ROLLPITCH = 0.00002;
    
    % coeffs for yaw correction
    Kp_YAW = 1.2;
    Ki_YAW = 0.00002;
    
    % rotation matrix of local coordinates to world coordinates
    % world_coord = DCM_Matrix * local_coord;
    % local_coord = DCM_Matrix' * world_coord;
    DCM_Matrix = eye(3);
    
    tm1 = 0;
    counter = 0;
    
    function [] = iter(anglesAhrs, gyroSrc, accsSrc, magSrc, offsetGyro, offsetAccel, currTime, timeDelta)
        %ITER single iteration of data from sensor
        % anglesAhrs  - angles from Arduino AHRS system
        % gyroSrc     - raw gyro data from sensor
        % accsSrc     - raw accel data from sensor
        % magSrc      - raw magnetometer data from sensor
        % offsetGyro  - gyro zero offsets, averaged during initialization
        % offsetAccel - accel zero offsets, constants
        % currTime    - total time of all iterations up to the current
        %               moment
        % timeDelta   - time step of current iteration
        
        % Data acquisition
        gyroCorrLoc = (GYRO_SIGN .* gyroSrc - offsetGyro);
        accsCorrLoc = (ACC_SIGN .* accsSrc - offsetAccel);

        counter = counter + 1;
        if counter > 5
            counter = 0;
            magCorrLoc = MAG_SIGN .* magSrc;
            % Calculate magnetometer heading
            MAG_Heading = magHeading(roll, pitch, magCorrLoc);
        end
        
        % DCM algorithm
        DCM_Matrix = matrixUpdate(gyroCorrLoc, Omega_I, Omega_P, timeDelta, DCM_Matrix);
        
        % Normalize
        DCM_Matrix = normalize(DCM_Matrix);
        
        % Drift correction
        [Omega_I, Omega_P] = driftCorrection(accsCorrLoc, DCM_Matrix, Omega_I, MAG_Heading);
        
        % Euler angles
        [roll, pitch, yaw] = eulerAngles(DCM_Matrix);
        
        times = [times; currTime];
        angDiffs = [angDiffs; [(anglesAhrs - radtodeg([roll pitch yaw]))]];
        
        tm2 = currTime;
        if (tm2 - tm1 >= 0.5)
            figure(f1);plot(times, angDiffs);
            tm1 = tm2;
        end
    end

    function [heading] = magHeading(roll, pitch, magCorrLoc)
        % scale mag values to +/-0.5 range
        magScaled = (magCorrLoc - MAG_SIGN .* MAG_MIN) ./ (MAG_MAX - MAG_MIN) - MAG_SIGN .* 0.5;
        % tilt compensated mag field X, Y
        magXY = [
            cos(pitch) sin(roll)*sin(pitch) cos(roll)*sin(pitch);
            0          cos(roll)            -sin(roll)
            ] * magScaled';
        heading = atan2(-magXY(2,1), magXY(1,1));
    end

    function [rad] = degtorad(deg)
        % Convert degrees to radians
        rad = deg .* pi() / 180.;
    end

    function [deg] = radtodeg(rad)
        % Convert degrees to radians
        deg = rad * 180. / pi();
    end

    function [y] = constrain(x, lowlimit, highlimit)
        y = min(max(x,lowlimit),highlimit);
    end

    function [roll, pitch, yaw] = eulerAngles(DCM_Matrix)
        pitch = -asin(DCM_Matrix(3,1));
        roll = atan2(DCM_Matrix(3,2), DCM_Matrix(3,3));
        yaw = atan2(DCM_Matrix(2,1), DCM_Matrix(1,1));
    end

    function [dcm] = normalize(DCM_Matrix)
        dcm = zeros(size(DCM_Matrix));
        error = - DCM_Matrix(1, :) * DCM_Matrix(2, :)' .* 0.5;
        temp = [
            DCM_Matrix(2, :) .* error + DCM_Matrix(1, :); 
            DCM_Matrix(1, :) .* error + DCM_Matrix(2, :)
            ];
        temp = [temp; cross(temp(1,:), temp(2, :))];
        for i = 1:3
            renorm = .5 * (3 - temp(i,:) * temp(i, :)');
            dcm(i,:) = temp(i, :) .* renorm;
        end
    end

    function [dcm] = matrixUpdate(gyroCorrLoc, Omega_I, Omega_P, timeDelta, DCM_Matrix)
        gyroScaled = degtorad(gyroCorrLoc .* GYRO_GAIN);
        Omega = gyroScaled + Omega_I + Omega_P;
        
        % Matrix update
        Update_Matrix = [
            0        -Omega(3)  Omega(2)
            Omega(3) 0         -Omega(1)
           -Omega(2) Omega(1)   0
            ] .* timeDelta;
        
        dcm = DCM_Matrix + DCM_Matrix * Update_Matrix;
    end

    function [Omega_I, Omega_P] = driftCorrection(accsCorrLoc, DCM_Matrix, Omega_I, MAG_Heading)
        % *** ROLL & PITCH ***
        
        % Scaled to gravity
        accelMagnitude = sqrt(accsCorrLoc * accsCorrLoc') / GRAVITY;
        
        % Dynamic weighting of accelerometer info (reliability filter)
        % Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
        accelWeight = constrain(1 - 2*abs(1 - accelMagnitude), 0, 1);    
        
        % Adjust the ground of reference
        errorRollPitch = cross(accsCorrLoc, DCM_Matrix(3,:));
        Omega_P = errorRollPitch * Kp_ROLLPITCH * accelWeight;
        Omega_I = Omega_I + errorRollPitch * Ki_ROLLPITCH * accelWeight;
        
        % *** YAW ***
        % We make the gyro YAW drift correction based on compass magnetic heading
        
        % calculating YAW error
        errorCourse = DCM_Matrix(1,1) * sin(MAG_Heading) - DCM_Matrix(2,1) * cos(MAG_Heading);
        % applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
        errorYaw = DCM_Matrix(3,:) .* errorCourse;
        % Adding Proportional
        Omega_P = Omega_P + errorYaw .* Kp_YAW;
        % Adding Integral
        Omega_I = Omega_I + errorYaw .* Ki_YAW;
    end
        
    serialLoop(@iter);

end

