function [ y ] = signal( x, freqs )
%SIGNAL Get a signal composed as sum of signals with given frequencies
    y = zeros(size(x));
    for i = 1:size(freqs,2)
        y = y + sin(freqs(1, i) * 2 * pi() * x);
    end
end

