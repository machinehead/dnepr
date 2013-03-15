function [] = plotMeanAccs()
    times = [];
    accs = [];
    meanAccs = [];
    
    accFlt = [];
    minAcc = 0;
    maxAcc = 0;
    mins = [];
    maxs = [];

    f1 = figure();
    f2 = figure();
    % f3 = figure();
    
    function [] = iter(anglesAhrs, gyroSrc, accsSrc, magSrc, offsetGyro, offsetAccel, currTime, timeDelta)
        accs = [accs; accsSrc];
        meanAccs = [meanAccs; mean(accs,1)];
        times = [times; currTime];
        mins = [mins; minAcc];
        maxs = [maxs; maxAcc];

        tm2 = currTime;
        if (tm2 - tm1 >= 0.5) 
            accFlt = lpf(accs(:,1)', 0.02, 2)';
            minAcc = min(accFlt);
            maxAcc = max(accFlt);
            figure(f1); plot(times, accs(:,1), 'b', times, accFlt, 'r', times, mins, 'g', times, maxs, 'g');
            % figure(f1);plot(times, accs(:,1), 'b', times, meanAccs(:,1), 'r');
            % figure(f2);plot(times, accs(:,2), 'b', times, meanAccs(:,2), 'r');
            % figure(f3);plot(times, accs(:,3), 'b', times, meanAccs(:,3), 'r');
            tm1 = tm2;
        end
    end

    tm1 = 0;
    serialLoop(@iter);

end
