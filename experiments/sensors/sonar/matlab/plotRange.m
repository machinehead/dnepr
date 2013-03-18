function [ ] = plotRange()
    times = [];
    ranges = [];
    
    fig1 = figure();
    
    tm1 = 0;
    
    function [] = iter(range, currTime, timeDelta)
        times = [times; currTime];
        ranges = [ranges; range];
        
        tm2 = currTime;
        if tm2 - tm1 > 0.5
            tm1 = tm2;
            figure(fig1);plot(times, ranges);
        end
    end

    serialLoop(@iter);

end

