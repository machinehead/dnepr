function [] = plotAccs()
    times = [0];
    accs = [];
    % z max + = 1073
    % z max - = -1023
    % y max + = 1035
    % y max - = 1024
    % x max + = 994
    % x max - = 1034
    
    % inv axis
    % z max = 1005
    % z min = -1090
    % y max = 1024
    % y min = -1035
    % x max = 1030
    % x min = -1001
    meanAccs = [1024 0 0 1024 0 0];
    accLens = [1024];

    f1 = figure();
    f2 = figure();
    f3 = figure();
    f4 = figure();
    
    K = 0.99;

    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        % static outlier elimination
        if abs(sqrt(accsLoc * accsLoc') - 1024) < 100
            accs = [accs; accsLoc];
            maxAccs = min(accs(max(1,size(accs,1)-50):size(accs,1),:), [], 1);
            row = [(K * meanAccs(size(meanAccs,1),1:3) + (1-K) * accsLoc) maxAccs];
            meanAccs = [meanAccs; row];
            times = [times; currTime];
            accLens = [accLens; sqrt(accsLoc * accsLoc')];

            tm2 = currTime;
            if (tm2 - tm1 >= 0.5) && (size(meanAccs,1) >= 1)
                figure(f1);plot(times, [round(meanAccs(:,1)) meanAccs(:,4)]);
                % figure(f1);plot(times(1:size(accs,1)), accs(:,1));
                % figure(f2);plot(times(1:size(accs,1)), accs(:,2));
                % figure(f3);plot(times(1:size(accs,1)), accs(:,3));
                figure(f2);plot(times, [round(meanAccs(:,2)) meanAccs(:,5)]);
                figure(f3);plot(times, [round(meanAccs(:,3)) meanAccs(:,6)]);
                figure(f4);plot(times, accLens);
                tm1 = tm2;
            end
        end
    end

    tm1 = 0;
    serialLoop(@iter);

end
