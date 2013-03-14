function [] = ahrs()
%AHRS Matlab port of Arduino AHRS by Pololu.

    times = [];
    angDiffs = [];
    f1 = figure();

    SENSOR_SIGN = [1 1 1 -1 -1 -1 1 1 1];
    GYRO_SIGN = SENSOR_SIGN(1:3); 
    ACC_SIGN = SENSOR_SIGN(4:6);
    MAG_SIGN = SENSOR_SIGN(7:9);

    MAG_MIN = [-883 -942 -533];
    MAG_MAX = [320 481 452];
    
    roll = 0;
    pitch = 0;
    yaw = 0;

    % L3G4200D gyro: 2000 dps full scale
    % 70 mdps/digit; 1 dps = 0.07
    GYRO_GAIN = 0.07;
    
    Omega_P = [0 0 0]; % Omega Proportional correction
    Omega_I = [0 0 0]; % Omega Integrator
    
    DCM_Matrix = eye(3);
    
    tm1 = 0;
    counter = 0;
    
    function [] = iter(anglesAhrs, gyroSrc, accsSrc, magSrc, offsetGyro, offsetAccel, currTime, timeDelta)
        % Data acquisition
        gyroCorrLoc = GYRO_SIGN .* (gyroSrc - offsetGyro);
        accsCorrLoc = ACC_SIGN .* (accsSrc - offsetAccel);

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
        
        % Euler angles
        [roll, pitch, yaw] = eulerAngles(DCM_Matrix);
        
        times = [times; currTime];
        angDiffs = [angDiffs; [anglesAhrs radtodeg([roll pitch yaw])]];
        
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
        heading = atan2(-magXY(1), magXY(2));
    end

    function [rad] = degtorad(deg)
        % Convert degrees to radians
        rad = deg .* pi() / 180.;
    end

    function [deg] = radtodeg(rad)
        % Convert degrees to radians
        deg = rad * 180. / pi();
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
            ] * timeDelta;
        
        dcm = DCM_Matrix + DCM_Matrix * Update_Matrix;
    end
        
    serialLoop(@iter);

end

