function [times, accs] = testIntegrateAccs(datasource)
    times = [];
    accs = [];
    
    f1 = figure();
    % f2 = figure();
    % f3 = figure();
    % f4 = figure();
    
    function [] = iter(gyro, accsSrc, mag, currTime, timeDelta, sonar, sonarNew)
        times = [times; currTime];
        accs = [accs; accsSrc];
    end

    datasource(@iter);
    
    figure(f1);
    plot(times, accs);
end
