% GENSIGNAL() - Generate sinusoidal signal with specified frequencies,
%       amplitudes, phase differences, signal-to-noise ratio, and time
%       delay
% 
% Usage
%       [signal,t,signals]=gensignal(freqs,amps,phases,Fs,duration,SNR,delay,plotsignal);
% 
% Inputs
%       freqs       =   desired frequencies (Hz)        [default: 1]
%       amps        =   desired amplitudes ("V")        [default: 1]
%       phases      =   desired phases (rad)            [default: random -pi a pi]
%       Fs          =   sampling rate (Hz)              [default: 1]
%       duration    =   duration of signal (s)          [default: 1]
%       SNR         =   signal-to-noise ratio (dB)      [default: Inf]
%       delay       =   time delay (s)                  [default: 0]
%       plotsignal  =   plot signal (0/1)               [default: 1]
% 
% Output   
%       signal      =   generated signal
%       t           =   time (s) vector 
%       signals     =   separate sine waves and noise
%
% Examples
% Simulate a 5 Hz sine wave:
%       signal = gensignal(5,1,[],1000,1,Inf,0);
% 
% Simulate two 5-Hz noisy signals with a 50-ms delay:
%       signal1 = gensignal(5,1,0,1000,1,3,0);
%       signal2 = gensignal(5,1,0,1000,1,3,.05);
% 
% Simulate "pink noise":
%       Fs = 1000;
%       freqs = linspace(1,Fs/4,Fs);
%       amps = exp(-.01*freqs);
%       [signal,t] = gensignal(freqs,amps,[],Fs,1,3,0);
% 
% Simulate time-variant 4-12 Hz signal:
%       Fs = 1000;
%       Signal = [];
%       for cycles = 1:100
%       freq = randsample(4:.5:12, 1);
%       cycleduration = (Fs/freq)/Fs;
%       signal = gensignal(freq,1,0,Fs,cycleduration);
%       Signal = [Signal signal];
%       end
% 
% Simulate 5 Hz - 80 Hz phase-amplitude coupled signal
%       [lf,t] = gensignal(5,1,0,1000,30); %low-frequency
%       [hf] = gensignal(80,1,0,1000,30); %high-frequency 
%       pacsignal = normalize(lf,'range') .* hf;
% 
% Simulate temporal interference signal
%       [signal1,t] = gensignal(5000,1,0,44100,10); 
%       signal2 = gensignal(5001,1,0,44100,10); %slightly higher frequency
%       signal = signal1+signal2;
% 
% Author: Danilo Benette Marques, 2018

function [signal,t,signals]=gensignal(freqs,amps,phases,Fs,duration,SNR,delay,plotsignal);

%Set default parameters 
% if fewer number or empty input arguments
if nargin==0 | isempty(freqs)
    freqs = 1;
end
if nargin<2 | isempty(amps)
    amps = ones(size(freqs));
end
if nargin<3 | isempty(phases)
    phases = (pi--pi)*rand(size(freqs))-pi; %gaussian noise has uniform phase distribution 
end
if nargin<4 | isempty(Fs)
    Fs = 1;
end
if nargin<5 | isempty(duration)
    duration = 1;
end
if nargin<6 | isempty(SNR)
    SNR = Inf;
end
if nargin<7 | isempty(delay)
    delay = 0;
end
if nargin<8 | isempty(plotsignal)
    plotsignal = 1;
end

%Time resolution and vector
dt = 1/Fs;
t=linspace(0,duration,duration*Fs);

%Generate sine waves for each frequency, amplitude, and phase
for i=1:length(freqs)
signals(i,:)=amps(i) * cos(2*pi*freqs(i)*t + phases(i));
end

%Sum sine waves 
signal=sum(signals,1);

%Time shift signal according to delay
signal=circshift(signal,round(delay*Fs),2);

%Generate noise with specified SNR
noise = randn(1,length(t))*sqrt(var(signal)/SNR); signals(end+1,:) = noise;

%Sum sine waves to noise
signal = signal+noise;

%Plot signal and FFT
if plotsignal
figure,
subplot(2,1,1)
plot(t,signal);
title('Generated signal')
ylabel('Amplitude')
xlabel('Time (s)')

N = length(signal);
df = Fs/N;
f = 0:df:Fs/2;

Sx = abs(fft(signal))/N;
Sx = Sx(1:N/2+1);
Sx(2:end-1) = 2*Sx(2:end-1);

subplot(2,1,2)
plot(f,Sx)
ylabel('Amplitude')
xlabel('Frequency (Hz)')
end

end

