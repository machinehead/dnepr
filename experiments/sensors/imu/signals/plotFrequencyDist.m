function [] = plotFrequencyDist( samples, sampleFrequency )
%PLOTFREQUENCYDIST Plot amplitude-frequency distribution of a signal.
%   Detailed explanation goes here

    L = size(samples,2);

    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
    Y = fft(samples, NFFT)/L;
    f = sampleFrequency/2*linspace(0,1,NFFT/2+1);

    % Plot single-sided amplitude spectrum.
    figure();
    plot(f,2*abs(Y(1:NFFT/2+1))) 
    title('Single-Sided Amplitude Spectrum of y(t)')
    xlabel('Frequency (Hz)')
    ylabel('|Y(f)|')
    
end

