function [ dcm ] = dcmd( angles )
    anglesLoc = num2cell(angles);
    [roll, pitch, yaw] = deal(anglesLoc{:});
    dcm = [cosd(pitch)*cosd(yaw)  sind(roll)*sind(pitch)*cosd(yaw)-cosd(roll)*sind(yaw)  cosd(roll)*sind(pitch)*cosd(yaw)+sind(roll)*sind(yaw);
           cosd(pitch)*sind(yaw)  sind(roll)*sind(pitch)*sind(yaw)+cosd(roll)*cosd(yaw)  cosd(roll)*sind(pitch)*sind(yaw)-sind(roll)*cosd(yaw);
           -sind(pitch)           sind(roll)*cosd(pitch)                                 cosd(roll)*cosd(pitch)];
end

