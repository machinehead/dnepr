function [accs, vels, times] = plotAccsVelsCoords()
    times = [];
    angles = [];
    accs = [];
    % accsKalman = [0 0 0];
    vels = [];
    % velsKalman = [0 0 0];
    coords = [];
    accGain = [1024./1013., 1024./1037., 1024./1053.];
    
    f1 = figure();
    % f2 = figure();
    f3 = figure();
    % f4 = figure();
    f5 = figure();

    K = 0.99;

    tm1 = clock();
    
    function [] = iter(anglesLoc, accsLoc, currTime, offsetGyro, offsetAccel)
        if currTime > 5
            dcm = dcmd(anglesLoc);
            angles = [angles; anglesLoc];
            % accsLoc = (accsLoc - offsetAccel) .* accGain - (dcm' * [0. 0. 1024.]')';
            accsWorld = (dcm * ((accsLoc - offsetAccel) .* accGain)')' -  [0. 0. 1024.];
            % accs = [accs; accsLoc];
            accs = [accs; accsWorld];
            % accsKalman = [accsKalman; K * accsKalman(size(accsKalman,1)) + (1-K) * accsLoc];

            times = [times; currTime];
            vels = cumsum(accs .* (9.81 / 1024.), 2) * 0.02;
            % velsKalman = cumtrapz(accsKalman .* (9.81 / 1024.), 2) * 0.02;
            coords = cumsum(vels) * 0.02;

            tm2 = clock();
            if etime(tm2,tm1) >= 1
                % figure(f1);plot(times, angles);
                figure(f1);plot(times, accs);title('Accelerations');
                % figure(f2);plot(times, accsKalman);title('Accelerations after Kalman filter');
                figure(f3);plot(times(1:size(vels,1)), vels);title('Rect integral of accs');
                % figure(f4);plot(times(1:size(velsKalman,1)), velsKalman);title('Trapezoid integral of Kalman accs');
                figure(f5);plot(times(1:size(coords,1)), coords);title('Rect 2nd order integral of accs');
                tm1 = tm2;
            end
        end
    end

    serialLoop(@iter);

end
