function [accs, meanAccs, vels, times, corrections] = integralAccCorrection()
    times = [];
    accs = [];
    meanAccs = [];
    vels = [];
    % coords = [0 0 0];

    f3 = figure();
    f4 = figure();
    % plot(times, vels);

    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        accsLoc = accsLoc - offsetAccel - [0. 0. 256.]; % - (dcm'*[0. 0. 256.]')';
        accs = [accs; accsLoc];
        meanAccs = [meanAccs; mean(accs,1)];
        itCount = itCount + 1;
        times = [times; itCount];
        vels = integrate(accs, 1);
        
        tm2 = clock();
        if etime(tm2,tm1) >= 1
            % figure(f3);plot(times(1:size(vels,1)), vels);
            figure(f3);plot(times(1:size(accs,1)), accs);
            figure(f4);plot(times(1:size(meanAccs,1)), meanAccs);
            tm1 = tm2;
        end
    end

    tm1 = clock();
    itCount = 0;
    serialLoop(@iter);
    
    finalVels = vels(size(vels,1),:);
    finalIter = times(size(times,1));
    corrections = finalVels/finalIter;

end
