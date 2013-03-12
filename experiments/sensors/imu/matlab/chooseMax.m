function [ topMax, hc ] = chooseMax( hc, edges )
%CHOOSEMAX Summary of this function goes here
%   Detailed explanation goes here

        topMax = [];
        
        for maxCount = 1:20
            [maxHC, ind] = max(hc);
            maxRow = [];
            for j = 1:size(maxHC,2)
                maxRow = [maxRow ind(1,j)];
                hc((ind(1,j)-5):(ind(1,j)+5),j) = 0;
            end
            disp(maxRow);
            topMax = [topMax; maxRow];
        end


end

