function [] = serialLoop( fun )
% SERIALLOOP ÷икл чтени€ данных с IMU, выполн€ющий на каждой итерации
% заданную функцию fun.
    port = serial('COM4','BaudRate',115200);
    port.Terminator = 'CR/LF';
    fopen(port);
    
    currTime = 0;

    while exist('runkey', 'file')
        l = fgetl(port);
        if strncmp(l, '!OFFSET,', 7)
            data = textscan(l, '!OFFSET,GYRO,%f,%f,%f,ACC,%f,%f,%f',1);
            offsetGyro = double([data{1,1:3}]);
            offsetAccel = double([data{1,4:6}]);
            disp(offsetGyro);
            disp(offsetAccel);
        elseif strncmp(l, '!ANG,', 5)
            data = textscan(l, '!ANG,%f,%f,%f,GYRO,%d,%d,%d,ACC,%d,%d,%d,MAG,%f,%f,%f,%f\r\n',1);
            if size(data{1,13},1)
                timeDelta = data{1,13};
                currTime = currTime + timeDelta;
                angles = double([data{1,1:3}]);
                gyro = double([data{1,4:6}]);
                accs = double([data{1,7:9}]);
                mag = double([data{1,10:12}]);
                fun(angles, gyro, accs, mag, offsetGyro, offsetAccel, currTime, timeDelta);
            end
        end
    end
    
    fclose(port);
    delete(port);
end

