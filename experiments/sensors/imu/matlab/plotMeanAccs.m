function [] = plotMeanAccs()
    times = [];
    accs = [];
    % meanAccs = [];
    
    accFlt = [];
    minAcc = [0 0 0];
    maxAcc = [0 0 0];
    mins = [];
    maxs = [];

    f1 = figure();
    f2 = figure();
    f3 = figure();
    
    function [] = iter(gyroSrc, accsSrc, magSrc, currTime, timeDelta)
        accs = [accs; accsSrc];
        % meanAccs = [meanAccs; mean(accs,1)];
        times = [times; currTime];
        mins = [mins; minAcc];
        maxs = [maxs; maxAcc];

        tm2 = currTime;
        if (tm2 - tm1 >= 0.5) 
            accFlt = lpf(accs, 0.02, 1);
            minAcc = min(accFlt);
            maxAcc = max(accFlt);
            disp(minAcc);
            disp(maxAcc);
            figure(f1); plot(times, accs(:,1), 'b', times, accFlt(:,1), 'r', times, mins(:,1), 'g', times, maxs(:,1), 'g');
            figure(f2); plot(times, accs(:,2), 'b', times, accFlt(:,2), 'r', times, mins(:,2), 'g', times, maxs(:,2), 'g');
            figure(f3); plot(times, accs(:,3), 'b', times, accFlt(:,3), 'r', times, mins(:,3), 'g', times, maxs(:,3), 'g');
            tm1 = tm2;
        end
    end

    tm1 = 0;
    serialLoop(@iter);

end
