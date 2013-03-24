function [] = plotAccsVelsCoords(datasource)
    times = [];
    % angles = [];

    accsWorld = [0 0 0];
    accsCurrWorld = [0 0 0];
    accsRaw = [];

    velsCurr = [0 0 0];
    vels = [];

    coordsCurr = [0 0 0];
    coords = [];
    
    sonarDisplacements = [];
    accDisplacements = [];
    
    sonarValueSum = 0;
    sonarReadCount = 0;
    sonarInitial = 0;
    
    sonarPrev = 0;
    sonarRepeatRead = 0;
    
    f2 = figure();
    f3 = figure();
    f4 = figure();
    f5 = figure();

    tm1 = 0;
    
    % timeStep = 0.01;
    
    ACC_CUTOFF_FREQ = 4;

    offsetAccel = [0 0 0];
    offsetAccelCount = 1;
    
    offsetAccelLocal = [0 0 0];
    offsetAccelLocalCount = 0;
    
    function [] = iter(anglesAhrs, dcm, accsCorrLoc, currTime, timeDelta, sonar, sonarNew)
        if currTime < 5
            sonarValueSum = sonarValueSum + sonar;
            sonarReadCount = sonarReadCount + 1;
            sonarInitial = sonarValueSum / sonarReadCount;
            tm1 = 0;
        else
            accsWorld = lpfUpdate((dcm * accsCorrLoc')' - [0. 0. 1024.], accsWorld, timeDelta, ACC_CUTOFF_FREQ);
            % accsWorld = (dcm * accsCorrLoc')' - [0. 0. 1024.];
            
            if sonarNew && sonar == sonarPrev
                sonarRepeatRead = sonarRepeatRead + 1;
            elseif sonarNew 
                sonarPrev = sonar;
                sonarRepeatRead = 1;

                offsetAccelLocal = [0 0 0];
                offsetAccelLocalCount = 0;
            end
            if sonarRepeatRead > 1
                offsetAccelLocal = (offsetAccelLocal * offsetAccelLocalCount + accsWorld) / (offsetAccelLocalCount + 1);
                offsetAccelLocalCount = offsetAccelLocalCount + 1;
            end
            if sonarNew && sonarRepeatRead > 3
                sonarRepeatRead = 0;

                offsetAccel = (offsetAccel * offsetAccelCount + offsetAccelLocal * offsetAccelLocalCount) / (offsetAccelCount + offsetAccelLocalCount);
                offsetAccelCount = offsetAccelCount + offsetAccelLocalCount;
                offsetAccelLocal = [0 0 0];
                offsetAccelLocalCount = 0;

                velsCurr = [0 0 0];
            end
            
            % angles = [angles; anglesAhrs];
            % accsCurrWorld = lpfUpdate((dcm * accsCorrLoc')' - [0. 0. 1024.] - offsetAccel, accsCurrWorld, timeDelta, ACC_CUTOFF_FREQ);
            accsCurrWorld = accsWorld - offsetAccel;
            
            accsRaw = [accsRaw; accsCurrWorld];
            
            velsCurr = velsCurr + accsCurrWorld .* (9.81 / 1024.0 * timeDelta);
            vels = [vels; velsCurr];
            
            coordsCurr = coordsCurr + velsCurr .* timeDelta;
            coords = [coords; coordsCurr];
            
            times = [times; currTime];
            sonarDisplacements = [sonarDisplacements; abs(sonar - sonarInitial) / 100];
            accDisplacements = [accDisplacements; sqrt(coordsCurr * coordsCurr')];

            tm2 = currTime;
            if tm2 - tm1 >= 1
                % figure(f1);plot(times, angles);title('Angles');

                figure(f2);plot(times, accsRaw);title('Accelerations in world -1g');
                figure(f3);plot(times, vels);title('Rect integral of accs');
                figure(f4);plot(times, coords);title('Rect 2nd order integral of accs');
                figure(f5);plot(times, [sonarDisplacements accDisplacements]);title('Sonar&accel displacements');
                
                tm1 = tm2;
            end
        end
    end

    ahrs(@iter, datasource);

end
