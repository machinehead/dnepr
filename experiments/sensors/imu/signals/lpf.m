function [ y ] = lpf( x, dt, cutoff )
%HPF Primitive low-pass filter
% x - time series in columns
    rc = 1 / (2 * pi() * cutoff);
    alpha = dt / (rc + dt);
    y = zeros(size(x));
    y(1,:) = x(1, :);
    for i = 2:size(x,1)
        y(i,:) = alpha * x(i, :) + (1 - alpha) * y(i-1, :);
    end
end

