function [] = serialLoopFast( fun )
% SERIALLOOP ÷икл чтени€ данных с IMU дл€ прошивки FastIMU,
%   выполн€ющий на каждой итерации заданную функцию fun.
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
        elseif strncmp(l, '!GYRO,', 6)
            data = textscan(l, '!GYRO,%d,%d,%d,ACC,%d,%d,%d,MAG,%d,%d,%d,%f,%d,%d\r\n',1);
            timeField = 10;
            sonarField = 11;
            lastField = 12;
            if size(data{1,lastField},1)
                timeDelta = data{1,timeField};
                currTime = currTime + timeDelta;
                gyro = double([data{1,1:3}]);
                accs = double([data{1,4:6}]);
                mag = double([data{1,7:9}]);
                sonar = double([data{1,sonarField}]);
                sonarNew = double([data{1,sonarField + 1}]);
                fun(gyro, accs, mag, currTime, timeDelta, sonar, sonarNew);
            else
                disp('Skip reading!');
            end
        end
    end
    
    fclose(port);
    delete(port);
end

