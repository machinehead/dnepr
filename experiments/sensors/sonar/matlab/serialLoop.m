function [] = serialLoop( fun )
% SERIALLOOP ÷икл чтени€ данных с сонара, выполн€ющий на каждой итерации
% заданную функцию fun.
    port = serial('COM4','BaudRate',115200);
    port.Terminator = 'CR/LF';
    fopen(port);
    
    currTime = 0;

    while exist('runkey', 'file')
        l = fgetl(port);
        if strncmp(l, '!Range:', 7)
            data = textscan(l, '!Range:%d,%f',1);
            range = data{1,1};
            timeDelta = data{1,2};
            currTime = currTime + timeDelta;
            disp(range);
            fun(range, currTime, timeDelta);
        end
    end
    
    fclose(port);
    delete(port);
end

