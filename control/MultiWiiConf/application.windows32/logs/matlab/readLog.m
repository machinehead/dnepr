function [times, raw_accel, raw_gyro, raw_mag, motors, rc_rpyts, auxes, alts, debugs, imu_rpys] = readLog( filename )
%READLOG Read the log of a MultiWiiConf run.
% invoke: [t,ra,rg,rm,m,rpyt,aux,alt,dbg,rpy] = readLog('2013_4_30_14_59_20.txt');
    times = [];
    raw_accel = [];
    raw_gyro = [];
    raw_mag = [];
    motors = [];
    rc_rpyts = [];
    auxes = [];
    alts = [];
    debugs = [];
    imu_rpys = [];

    motor = [0 0 0 0];
    rc_rpyt = [0 0 0 0];
    aux = [0 0 0 0];
    
    fid = fopen(filename);
    currTimeUS = 0;

    tline = fgetl(fid);
    while ischar(tline)
        if strncmp(tline, 'STATUS;', 7)
            if currTimeUS > 0
                times = [times; currTimeUS];
                raw_accel = [raw_accel; accelData];
                raw_gyro = [raw_gyro; gyroData];
                raw_mag = [raw_mag; magData];
                motors = [motors; motor];
                rc_rpyts = [rc_rpyts; rc_rpyt];
                auxes = [auxes; aux];
                alts = [alts; alt];
                debugs = [debugs; dbg];
                imu_rpys = [imu_rpys; imu_rpy];
            end
            
            data = textscan(tline, 'STATUS;CYCLETIME;%d;I2CERR;%d;PRESENT;%d;MODE;%d', 1, 'ReturnOnError', false);
            cycleTimeUS = data{1,1};
            currTimeUS = currTimeUS + cycleTimeUS;
        elseif strncmp(tline, 'RAW_IMU;', 8)
            data = textscan(tline, 'RAW_IMU;AX;%f;AY;%f;AZ;%f;GX;%f;GY;%f;GZ;%f;MAGX;%f;MAGY;%f;MAGZ;%f', 1, 'ReturnOnError', false);
            accelData = double([data{1,1:3}]);
            gyroData = double([data{1,4:6}]);
            magData = double([data{1,7:9}]);
        elseif strncmp(tline, 'MOT;', 4)
            data = textscan(tline, 'MOT;0;%f;1;%f;2;%f;3;%f;4;0.0;5;0.0;6;0.0;7;0.0;', 1, 'ReturnOnError', false);
            motor = double([data{1,1:4}]);
        elseif strncmp(tline, 'RC;', 3)
            data = textscan(tline, 'RC;ROLL;%f;PITCH;%f;YAW;%f;THROTTLE;%f;AUX1;%f;AUX2;%f;AUX3;%f;AUX4;%f', 1, 'ReturnOnError', false);
            rc_rpyt = double([data{1,1:4}]);
            aux = double([data{1,5:8}]);
        elseif strncmp(tline, 'ALTITUDE;', 9)
            data = textscan(tline, 'ALTITUDE;%f', 1, 'ReturnOnError', false);
            alt = data{1,1};
        elseif strncmp(tline, 'DEBUG;', 6)
            data = textscan(tline, 'DEBUG;DEBUG1%f;DEBUG2;%f;DEBUG3;%f;DEBUG4;%f', 1, 'ReturnOnError', false);
            dbg = double([data{1,1:4}]);
        elseif strncmp(tline, 'ATTITUDE;', 6)
            data = textscan(tline, 'ATTITUDE;ANGX;%f;ANGY;%f;HEAD;%f', 1, 'ReturnOnError', false);
            imu_rpy = double([data{1,1:3}]);
        end
        
        tline = fgetl(fid);
    end

    fclose(fid);

end

