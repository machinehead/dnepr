function [accs, vels, times] = plotAccsVelsCoords()
    times = [];
    angles = [];
    accs = [];
    vels = [];
    coords = [];
    accGain = [1024./1005.5, 1024./1025.5, 1024./1042.];
    
    f1 = figure();
    f2 = figure();
    f3 = figure();
    % f4 = figure();
    f5 = figure();

    tm1 = clock();
    
    function [] = iter(anglesAhrs, gyroSrc, accsSrc, magSrc, offsetGyro, offsetAccel, currTime, timeDelta)
        if currTime > 5
            dcm = dcmd(anglesAhrs);
            angles = [angles; anglesAhrs];
            accsWorld = (dcm * ((accsSrc - offsetAccel) .* accGain)')' - [0. 0. 1024.];
            accs = [accs; accsWorld];

            times = [times; currTime];
            vels = cumsum(accs .* (9.81 / 1024.), 2) * 0.02;
            coords = cumtrapz(vels, 1) * 0.02;

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

    serialLoop(@iter);

end
