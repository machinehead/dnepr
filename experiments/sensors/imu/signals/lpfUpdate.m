function [ y ] = lpfUpdate( x, yPrev, dt, cutoff )
%HPF Primitive low-pass filter
% x - time series in columns
    rc = 1 / (2 * pi() * cutoff);
    alpha = dt / (rc + dt);
    y = alpha * x + (1 - alpha) * yPrev;
end

