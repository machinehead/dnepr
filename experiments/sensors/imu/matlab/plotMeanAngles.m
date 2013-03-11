function [] = plotMeanAngles()
    times = [];
    angles = [];
    meanAngles = [];

    f1 = figure();
    f2 = figure();
    f3 = figure();
    movAvgRange = 300;

    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        angles = [angles; anglesLoc];
        if size(angles,1) > movAvgRange
            meanAngles = [meanAngles; mean(angles(size(angles,1)-movAvgRange:size(angles,1),:),1)];
            times = [times; currTime];
        end
        
        tm2 = clock();
        if (etime(tm2,tm1) >= 1) && (size(meanAngles,1) >= 1)
            figure(f1);plot(times, meanAngles(:,1));
            figure(f2);plot(times, meanAngles(:,2));
            figure(f3);plot(times, meanAngles(:,3));
            tm1 = tm2;
        end
    end

    tm1 = clock();
    serialLoop(@iter);

end
