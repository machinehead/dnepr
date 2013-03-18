function [accs, vels, times] = plotAccsVelsCoords()
    times = [];
    angles = [];
    accs = [];
    vels = [];
    coords = [];
    
    f1 = figure();
    f2 = figure();
    f3 = figure();
    % f4 = figure();
    f5 = figure();

    tm1 = clock();
    
    timeStep = 0.02;
    
    function [] = iter(anglesAhrs, dcm, accsCorrLoc, currTime, timeDelta)
        if currTime > 5
            angles = [angles; anglesAhrs];
            accsWorld = (dcm * accsCorrLoc')' - [0. 0. 1024.];
            accs = [accs; accsWorld];

            times = [times; currTime];
            vels = cumsum(accs .* (9.81 / 1024.), 2) * timeStep;
            coords = cumtrapz(vels, 1) * timeStep;

            tm2 = clock();
            if etime(tm2,tm1) >= 1
                figure(f1);plot(times, angles);title('Angles');
                figure(f2);plot(times, accs);title('Accelerations');
                figure(f3);plot(times(1:size(vels,1)), vels);title('Rect integral of accs');
                figure(f5);plot(times(1:size(coords,1)), coords);title('Rect 2nd order integral of accs');
                tm1 = tm2;
            end
        end
    end

    ahrs(@iter);

end
