function [] = plotMeanAccs()
    times = [];
    accs = [];
    meanAccs = [];

    f1 = figure();
    f2 = figure();
    f3 = figure();
    
    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        accs = [accs; accsLoc];
        meanAccs = [meanAccs; mean(accs,1)];
        times = [times; currTime];

        tm2 = currTime;
        if (tm2 - tm1 >= 0.5) 
            figure(f1);plot(times, accs(:,1), 'b', times, meanAccs(:,1), 'r');
            figure(f2);plot(times, accs(:,2), 'b', times, meanAccs(:,2), 'r');
            figure(f3);plot(times, accs(:,3), 'b', times, meanAccs(:,3), 'r');
            tm1 = tm2;
        end
    end

    tm1 = 0;
    serialLoop(@iter);

end
