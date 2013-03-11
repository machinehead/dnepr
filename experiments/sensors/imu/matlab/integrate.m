function [ ints ] = integrate( Y, xUnitSize )
%INTEGRATE Cumulative trapezoid integral
%   
    ints = zeros(size(Y));
    for i = 1:size(Y,2)
        ints(:,i) = cumtrapz(double(Y(:,i)) * xUnitSize);
    end;
end

