function [ sonarUpd ] = addAccel( sonar, accel, timescale )
    spd = 0;
    dist = sonar(1);
    
    sonarUpd = [dist];
    sonarChanged = 1;
    
    for i = 2:size(sonar,1)
        if sonar(i) ~= sonar(i-1)
            dist = sonar(i);
            spd = (sonar(i) - sonar(sonarChanged)) / (timescale * sonarChanged - i);
            sonarChanged = i;
        end;
        spd = spd + accel(i) * 981.0 / 256.0 * timescale;
        dist = dist + spd * timescale;
        sonarUpd = [sonarUpd; dist];
    end;

end

