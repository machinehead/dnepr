function [ y ] = mwiiLpfPrev( x )
%MWIILPFPREV 
%   First version of MultiWii acc LPF in our github repo.
% x - time series in columns
    lpf_factor = 4;

    y = zeros(size(x));

    accLPF32 = zeros(1,size(x,2));
    y(1,:) = x(1, :);
    for i = 2:size(x,1)
        accLPF32 = accLPF32 - accLPF32 / pow2(lpf_factor);
        accLPF32 = accLPF32 + x(i, :);
        y(i,:) = accLPF32 / pow2(lpf_factor);
    end
end

