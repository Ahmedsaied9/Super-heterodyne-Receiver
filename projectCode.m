%reading the two signals and their Sampling Frequency and storing their values

[m1_twoChannel,Fs_m1]=audioread('Short_FM9090.wav');
[m2_twoChannel,Fs_m2]=audioread('Short_QuranPalestine.wav');
 
%Converting signals to single channel
m1_oneChannel= m1_twoChannel(:,1)+ m1_twoChannel(:,2);%adding the first and second columns to get a single channel audio 

m2_oneChannel= m2_twoChannel(:,1)+ m2_twoChannel(:,2);%adding the first and second columns to get a single channel audio
 
BW=1e+04; %definig the bandwidth of both signals neglecting any higher frequency components since they have a small magnitude

%interpolationg Signals to increase number of samples
m1=interp(m1_oneChannel,10);
m2=interp(m2_oneChannel,10);
Fs_m1=Fs_m1*10;
Fs_m2=Fs_m2*10;
 
%Zero padding the m1 to be the same length of m2 
m1=[m1' zeros(1,416640)]';
 
N=length(m1); %defining the length of x-axis to be the nearest power of 2 to the length of message signals m1 and m2
 
 
M1=fft(m1,N);%fourier transform of m1 
M2=fft(m2,N);%fourier transform of m2
f=[-N/2:(N/2)-1]*(Fs_m1/N); %defining frequency vector  

 
%Carrier signals c1 and c2
t=linspace(0,N/Fs_m1,N);%defining time vector to be the same length of m1 and m2
Fc_o=100000;%first carrier frequency
delta_F=50e+03;
Fc_1=Fc_o+delta_F;%second carrier frequency
c1=cos(2*pi*Fc_o*t);%first carrier of frequency 100KHz
c2=cos(2*pi*Fc_1*t);%second carrier of frequency 150KHz
 
%DSB-SC Modulator
m1_modulated=m1.*c1'; %multiplying the first message m1 with the carrier signal c1
m2_modulated=m2.*c2'; %multiplying the second message m2 with the carrier signal c2
 
M1_modulated=fft(m1_modulated);%Applying Fourier Transform to the modulated version of m1
M2_modulated=fft(m2_modulated);%Applying Fourier Transform to the modulated version of m2

 
%FDM the two signals
m_FDM=m1_modulated+m2_modulated; %adding the two modulated signals in time domain
M_FDM=fft(m_FDM); %fourier transform of the summation of the two modulated message signals to get the spectrum of the frequency division multiplexed signals

%End of Transmitter stage
 



%Reciever
 
signal=input('Which file do you want to hear?(1,2)');%asking the user to choose one of the two signals to hear

%RF Stage
 
%designing a BPF tuned at the desired signal frequency to extract the desired signal

%tuning the BPF to be centred at the frequency of the desired signal
if(signal==1)
    Fc=Fc_o;
else
    Fc=Fc_1;
end

%definig the cutoff frequencies of the filter
F_cutoff2=(Fc+BW)*2/(Fs_m1);
F_cutoff1=(Fc-BW)*2/(Fs_m1);
B1 = fir1(128,[F_cutoff1,F_cutoff2],'bandpass');%the coefficients of the BPF
Chosen_Signal_after_RF_stage=filter(B1,[1],m_FDM); %apply the filter to the FDM  signal to reject any image frequency

 
%Oscillator
F_IF=25e+03;%defining the intermediate frequency(IF)=25KHz
Fc3=Fc+F_IF;%defining the carrier frequency 
c3=cos(2*pi*Fc3*t);%carrier of frequency=Wn+Wif
 
%Mixer
Chosen_Signal_IF=Chosen_Signal_after_RF_stage.*c3';


%IF stage
 
%designing a BPF tuned at centre frequency 25KHz
F_cutoff6=(F_IF+BW)*2/(Fs_m1);
F_cutoff5=(F_IF-BW)*2/(Fs_m1);
B3 = fir1(128,[F_cutoff5,F_cutoff6],'bandpass');
 
Chosen_Signal_after_IF_stage=filter(B3,[1],Chosen_Signal_IF); %apply a BPF centred at IF 25KHz to extract the desired signal at IF 

 
 
 
%Baseband Demodulator
 
c5=cos(2*pi*F_IF*t);%carrier signal of frequency 25KHz
 
%designing a LPF of cuttoff frequency equal to the signal bandwidth at baseband 
Cuttoff_Frequency= BW*2/Fs_m1;
B_LPF= fir1(128, Cuttoff_Frequency,'low');
 
Chosen_Signal_at_baseband=Chosen_Signal_after_IF_stage.*c5';%multiplying the desired signal with a carrier signal of frequency 25KHz to shift the desired signal to baseband

 

 
Chosen_Signal_demodulated=filter(B_LPF,[1],Chosen_Signal_at_baseband);%applying LPF to baseband signal to extract demodulated desired message signal

 

 
% Resample the demodulated signal at its original rate (Fs=44100)
Chosen_Signal_demodulated_resampled= decimate(Chosen_Signal_demodulated,10);

 
%Playing the demodulated audio file 
sound(Chosen_Signal_demodulated_resampled,44100);

%plotting the transmitter stage graphs
figure;
subplot(511);
plot(f,fftshift(abs(M1)));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('m1 in frequency domain');
subplot(512);
plot(f,fftshift(abs(M2)));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('m2 in frequency domain');
subplot(513);
plot(f,fftshift(abs(M1_modulated)));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('modulated signal m1 in frequency domain');
subplot(514);
plot(f,fftshift(abs(M2_modulated)));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('modulated signal m2 in frequency domain');
subplot(515);
plot(f,fftshift(abs(M_FDM)));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('modulated FDM signals in frequency domain');


%plotting the Reciever stages outputs' graphs
figure;
subplot(511);
plot(f,fftshift(abs(fft(Chosen_Signal_after_RF_stage))));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('Chosen Signal after RF stage');

subplot(512);
plot(f,fftshift(abs(fft(Chosen_Signal_IF))));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('Chosen Signal after the mixer of frequency=Wc+Wif');

subplot(513);
plot(f,fftshift(abs(fft(Chosen_Signal_after_IF_stage))));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('Chosen Signal after IF stage');

subplot(514);
plot(f,fftshift(abs(fft(Chosen_Signal_at_baseband))));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('Chosen Signal at baseband');

subplot(515);
plot(f,fftshift(abs(fft(Chosen_Signal_demodulated))));
xlabel('Frequency(Hz)');
ylabel('Magnitude');
grid on;
title('Chosen Signal Recovered');







