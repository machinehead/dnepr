function [angles, rawAngles, rawAccel, rawMag] = ahrsRead(filename)
    fileID = fopen(filename);
    data = textscan(fileID, '%f,%f,%f,GYRO,%d,%d,%d,ACC,%d,%d,%d,MAG,%f,%f,%f\r\n',100500);
    angles = [data{1,1:3}];
    rawAngles = [data{1,4:6}];
    rawAccel = [data{1,7:9}];
    rawMag = [data{1,10:12}];
    fclose(fileID);
end
