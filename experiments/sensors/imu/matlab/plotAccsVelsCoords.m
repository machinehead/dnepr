function [accs, vels, times] = plotAccsVelsCoords()
    times = [0];
    angles = [0 0 0];
    accs = [0 0 0];
    accsKalman = [0 0 0];
    accsMovAvg = [0 0 0];
    vels = [0 0 0];
    velsMovAvg = [0 0 0];
    velsKalman = [0 0 0];
    % coords = [0 0 0];
    accGain = [1024/1015 1024/1029.5 1024/1048];
    
    f1 = figure();
    f2 = figure();
    f3 = figure();
    f5 = figure();

    K = 0.99;

    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        if currTime > 5
            dcm = dcmd(anglesLoc);
            angles = [angles; anglesLoc];
            accsLoc = (accsLoc - offsetAccel - [40 -20 -70]) .* accGain - (dcm' * [0. 0. 1024.]')';
            accs = [accs; accsLoc];
            accsKalman = [accsKalman; K * accsKalman(size(accsKalman,1)) + (1-K) * accsLoc];
            accsAvg = mean(accs(1:max(1,size(accs,1)-accMovAvgSamples+1),:),1);
            accsMovAvg = [accsMovAvg; accsAvg];

            times = [times; currTime];
            vels = integrate(accs .* (2 * 9.81 / 2048.), 0.02);
            velsMovAvg = integrate(accsMovAvg .* (2 * 9.81 / 2048.), 0.02);
            velsKalman = integrate(accsKalman .* (2 * 9.81 / 2048.), 0.02);
            % coords = integrate(vels, 0.02);

            tm2 = clock();
            if etime(tm2,tm1) >= 1
                figure(f1);plot(times, angles);
                figure(f2);plot(times, accs);
                figure(f3);plot(times, accsKalman);
                % figure(f3);plot(times(1:size(vels,1)), vels);
                % figure(f4);plot(times(1:size(velsMovAvg,1)), velsMovAvg);
                figure(f5);plot(times(1:size(velsKalman,1)), velsKalman);
                % figure(f4);plot(times(1:size(coords,1)), coords);
                tm1 = tm2;
            end
        end
    end

    tm1 = clock();
    accMovAvgSamples = 10;
    serialLoop(@iter);

end
