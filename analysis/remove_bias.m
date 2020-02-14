function out = remove_bias(in, threshold, window_size)

    signal = in;
    
    signal2 = in;

    count = 0;
    start = 1;
    for i = 2:length(signal)
        if count == 0
            start = i;
        end
        diff = signal(i) - signal(i-1);
        if abs(diff) < threshold
            count = count + 1;
        else
            count = 0;
        end

        if count >= window_size
            m = mean(signal(start:i,:));
            signal(start:end,:) = signal(start:end,:) - m;
%             signal(start:i,:) = zeros(length(signal(start:i,:)),1);
            count = 0;
        end

    end
    
    count = 0;
    start = 1;
    for i = 2:length(signal)
        if count == 0
            start = i;
        end
        diff = signal(i) - signal(i-1);
        if abs(diff) < threshold
            count = count + 1;
        else
            count = 0;
        end

        if count >= window_size
            signal2(start:i,:) = zeros(length(signal2(start:i,:)),1);
            count = 0;
        end

    end

    out = signal2;

end