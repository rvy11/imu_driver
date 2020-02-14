% Plot and return FFT of signal
function fft_out = plot_fft(x, Fs, L)
    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
    X = fft(x,NFFT)/L;
    f = Fs/2*linspace(0,1,NFFT/2+1);

    % Plot single-sided amplitude spectrum.
    plot(f,2*abs(X(1:NFFT/2+1))) 
    title('Single-Sided Amplitude Spectrum of x(t)')
    xlabel('Frequency (Hz)')
    ylabel('|X(f)|')
    fft_out = X;
%     Y = fft(X);
%     P2 = abs(Y/L);
%     P1 = P2(1:L/2+1);
%     P1(2:end-1) = 2*P1(2:end-1);
%     f = Fs*(0:(L/2))/L;
%     plot(f,P1); 
%     fft_out = Y;
end