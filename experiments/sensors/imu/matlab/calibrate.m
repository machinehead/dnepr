function [] = calibrate()
    times = [];
    means = [];
    accs = [];
    % lens = [];

    f1 = figure();
    f2 = figure();
    f3 = figure();
    f4 = figure();
    
    xTol = 7;
    yTol = 7;
    zTol = 5;
    % lenTol = 1.;
    accsMean = [0 0 0];

    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        accsLoc = accsLoc ./ 4;
        if size(accs,1) < 200
            pass = (abs(accsLoc(1,3) - 64) <= zTol) && (abs(accsLoc(1,1) - 0) <= xTol) && (abs(accsLoc(1,2) - 0) <= yTol);
        else
            accsOffset = accsLoc - accsMean + [0. 0. 64.];
            pass = (abs(accsOffset(1,3) - 64) <= zTol / 2) && (abs(accsOffset(1,1) - 0) <= xTol / 2) && (abs(accsOffset(1,2) - 0) <= yTol / 2);
            % len = sqrt(accsOffset * accsOffset');
            % times = [times; currTime];
            % lens = [lens; len];
            % disp(len);
            % pass = abs(len - 64.) < lenTol;
        end
        if pass
            disp(accsLoc);
            accs = [accs; accsLoc];
            accsMean = mean(accs, 1);
            times = [times; currTime];
            means = [means; accsMean];
        end
        
        
        tm2 = clock();
        if etime(tm2,tm1) >= 1
            if(size(accs,2) >= 3)
                figure(f1);hist(accs(:,1),-xTol:xTol);
                figure(f2);hist(accs(:,2),-yTol:yTol);
                figure(f3);hist(accs(:,3),-(64+zTol):64+zTol);
            end
            if size(means,1) > 1
               figure(f4);plot(times,means);
            end
            tm1 = tm2;
        end
    end

    tm1 = clock();
    serialLoop(@iter);

end
