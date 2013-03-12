function [accs] = accsHist()
    accs = [];

    f1 = figure();
    f2 = figure();
    f3 = figure();
   
    edges = -2048:2048;
    
    tm1 = 0;
    
    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        accs = [accs; accsLoc];

        tm2 = currTime;
        if (tm2 - tm1 >= 0.5) 
            figure(f1);hist(accs(:,1), -2048:2048);
            figure(f2);hist(accs(:,2), -2048:2048);
            figure(f3);hist(accs(:,3), -2048:2048);
            tm1 = tm2;
        end
    end

    serialLoop(@iter);

end
