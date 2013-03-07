port = serial('COM4','BaudRate',115200);
port.Terminator = 'CR/LF';
fopen(port);

times = [0];
angles = [0 0 0];
accs = [0 0 0];
vels = [0 0 0];
coords = [0 0 0];
f1 = figure();
plot(times, angles);
f2 = figure();
plot(times, accs);
f3 = figure();
plot(times, vels);
f4 = figure();
plot(times, coords);

currTime = 0;
tm1 = clock();

accMovAvgSamples = 10;

while exist('runkey', 'file')
  l = fgetl(port);
  if strncmp(l, '!OFFSET,', 7)
      data = textscan(l, '!OFFSET,GYRO,%f,%f,%f,ACC,%f,%f,%f',1);
      offsetGyro = [data{1,1:3}];
      offsetAccel = [data{1,4:6}];
  elseif strncmp(l, '!ANG,', 5)
      data = textscan(l, '!ANG,%f,%f,%f,GYRO,%d,%d,%d,ACC,%d,%d,%d,MAG,%f,%f,%f,%f\r\n',1);
      if size(data{1,13},1)
        currTime = currTime + data{1,13};
        anglesLocal = num2cell([data{1,1:3}]);
        [roll, pitch, yaw] = deal(anglesLocal{:});
        dcm = [cosd(pitch)*cosd(yaw)  sind(roll)*sind(pitch)*cosd(yaw)-cosd(roll)*sind(yaw)  cosd(roll)*sind(pitch)*cosd(yaw)+sind(roll)*sind(yaw);
               cosd(pitch)*sind(yaw)  sind(roll)*sind(pitch)*sind(yaw)+cosd(roll)*cosd(yaw)  cosd(roll)*sind(pitch)*sind(yaw)-sind(roll)*cosd(yaw);
               -sind(pitch)           sind(roll)*cosd(pitch)                                 cosd(roll)*cosd(pitch)];
        angles = [angles; cell2mat(anglesLocal)];
        accsLocal = double([data{1,7:9}]) - offsetAccel + (dcm'*[0. 0. 256.]')';
        if size(accs,1) >= accMovAvgSamples - 1
            prevAccsSum = sum(accs((size(accs,1)-accMovAvgSamples+2):size(accs,1),:),1);
            accsLocal = (accsLocal + prevAccsSum) ./ accMovAvgSamples;
        end
        accs = [accs; accsLocal];
        times = [times; currTime];
        vels = integrate(accs .* (8 * 9.81 / 2048.), 0.02);
        coords = integrate(vels, 0.02);
      end
  end
  tm2 = clock();
  if etime(tm2,tm1) >= 1
      figure(f1);plot(times, angles);
      figure(f2);plot(times, accs);
      figure(f3);plot(times(1:size(vels,1)), vels);
      figure(f4);plot(times(1:size(coords,1)), coords);
      tm1 = tm2;
  end
end

fclose(port);
delete(port);
clear port

