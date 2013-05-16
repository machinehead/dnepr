function [ y ] = mwiiLpfNew( x )
%MWIILPFPREV 
%   First version of MultiWii acc LPF in our github repo.
% x - time series in columns
    lpf_factor = 2;

    y = zeros(size(x));

    accLPF = zeros(1,size(x,2));
    y(1,:) = x(1, :);
    for i = 2:size(x,1)
        accLPF = accLPF * (1 - 1 / lpf_factor) + x(i, :) / lpf_factor;
        y(i,:) = accLPF;
    end
end

