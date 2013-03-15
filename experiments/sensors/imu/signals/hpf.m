function [ y ] = hpf( x, dt, cutoff )
%HPF Primitive high-pass filter
% x - time series in columns
    rc = 1 / (2 * pi() * cutoff);
    alpha = rc / (rc + dt);
    y = zeros(size(x));
    y(1, :) = x(1, :);
    for i = 2:size(x,1)
        y(i, :) = alpha*(y(i-1, :) + x(i, :) - x(i-1, :));
    end
end


