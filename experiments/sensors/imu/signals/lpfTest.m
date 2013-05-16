% control loop runs at about 300Hz
measureFreq = 300;

freqs = [1:3:100];
xs = 1:(1/measureFreq):100;
sign = signal(xs,freqs);
plotFrequencyDist(sign, measureFreq);
plotFrequencyDist(lpf(sign', (1/measureFreq), 5)', measureFreq);
plotFrequencyDist(mwiiLpfPrev(sign')', measureFreq);
plotFrequencyDist(mwiiLpfNew(sign')', measureFreq);
