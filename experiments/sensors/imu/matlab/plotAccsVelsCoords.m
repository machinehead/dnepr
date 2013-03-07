function [] = plotAccsVelsCoords()
    times = [0];
    angles = [0 0 0];
    accs = [0 0 0];
    vels = [0 0 0];
    coords = [0 0 0];

    f1 = figure();
    plot(times, angles);
    f2 = figure();
    plot(times, accs);
    f3 = figure();
    plot(times, vels);
    f4 = figure();
    plot(times, coords);

    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        dcm = dcmd(anglesLoc);
        angles = [angles; anglesLoc];
        accsLoc = accsLoc - offsetAccel + (dcm'*[0. 0. 256.]')';
        if size(accs,1) >= accMovAvgSamples - 1
            prevAccsSum = sum(accs((size(accs,1)-accMovAvgSamples+2):size(accs,1),:),1);
            accsLoc = (accsLoc + prevAccsSum) ./ accMovAvgSamples;
        end
        accs = [accs; accsLoc];
        times = [times; currTime];
        vels = integrate(accs .* (8 * 9.81 / 2048.), 0.02);
        coords = integrate(vels, 0.02);
        
        tm2 = clock();
        if etime(tm2,tm1) >= 1
            figure(f1);plot(times, angles);
            figure(f2);plot(times, accs);
            figure(f3);plot(times(1:size(vels,1)), vels);
            figure(f4);plot(times(1:size(coords,1)), coords);
            tm1 = tm2;
        end
    end

    tm1 = clock();
    accMovAvgSamples = 10;
    serialLoop(@iter);

end
