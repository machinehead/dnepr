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
                currTime = currTime + data{1,13};
                angles = double([data{1,1:3}]);
                accs = double([data{1,7:9}]);
                fun(angles,accs,currTime,offsetGyro,offsetAccel);
            end
        end
    end
    
    fclose(port);
    delete(port);
end

