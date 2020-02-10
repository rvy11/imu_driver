% Plot and return FFT of signal
function fft_out = plot_fft(X, Fs, L)
%     fig = figure;
    Y = fft(X);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L/2))/L;
    plot(f,P1); 
    fft_out = Y;
end