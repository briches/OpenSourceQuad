function yfilt = filtfft(y,dt,fmin,fmax,meanrem,plotting)
%
% function yfilt = filtfft(y,dt,fmin,fmax,meanrem,plotting)
%
% - performs simple frequency domain filtering by removing spectral lines from
%   a spectrum and performing an inverse FFT
% - returns filtered data and has the option to plot both the time and 
%   frequency domain traces before and after filtering
%
% input:       y - data vector
%             dt - spacing of signal in time domain
%      fmin,fmax - zero frequency components from fmin to fmax
%        meanrem - mean removal (0:no, 1:yes) - default no
%       plotting - plotting     (0:no, 1:yes) - default yes
%
% output: yfilt - filtered data
%         option to plot signal in time & frequency domain before & after filtering

% set some defaults

if nargin == 0, fprintf('\n  Error: specify an input data vector!\n\n'); return; end
if nargin == 1, dt=1; fmin=0; fmax=fmin; meanrem=0; plotting=1; end
if nargin == 2,       fmin=0; fmax=fmin; meanrem=0; plotting=1; end
if nargin == 3,               fmax=fmin; meanrem=0; plotting=1; end
if nargin == 4,                          meanrem=0; plotting=1; end
if nargin == 5,                                     plotting=1; end

blocksize = length(y);
if (blocksize/2 - floor(blocksize/2)) ~= 0  % check to see if blocksize is even
  fprintf('\n  NOTE: reduced data block length by one to make an even number in length\n\n');
  y = y(1:end-1);
  blocksize = blocksize - 1;
end

if meanrem == 1                   % mean removal
  y = y - mean(y);
end

fftraw = fft(y);                  % perform FFT

numfft = length(fftraw)/2 + 1;    % number of spectral lines
freqmax = 1/dt/2;                 % maximum frequency
if (fmin < 0 || fmax > freqmax)    % check out frequency range
  fprintf('\n\n ERROR: fmin or fmax out of range\n\n');
  return;
end
if fmin > fmax;                   % swap if wrong order
  temp = fmin; fmin = fmax; fmax = temp; 
end; 

df = freqmax/(numfft - 1);
indexmin = round(fmin/df) + 1;    % indices of componants to zero out
indexmax = round(fmax/df) + 1;

fftfiltered = fftraw;

fftfiltered(indexmin:indexmax) = 0;  % eliminate indicated frequency componants
if indexmin == 1; indexmin = indexmin + 1; end;  % special case for zero freq comp
fftfiltered(blocksize-indexmax+2:blocksize-indexmin+2) = 0;  % negative freqs too!

yfilt = real(ifft(fftfiltered));  % return filtered signal back to time domain
                                  % ignore tiny imaginary components
if plotting == 1
%  figure
  % time domain plot of original signal
  
  subplot(4,1,1);
  t = 0:dt:(blocksize-1)*dt;  % create time vector for plotting
  plot(t,y);
  xlabel('time (sec)');
  ylabel('ampl (orig)');
  title('FILTFFT: FFT Filtering - results in time & freq domain');

  % for the FFT plot, only plot length(ffty)/2 + 1 points (i.e. one sided)
  % and scale x-axis as frequency...

  subplot(4,1,2);
  f = linspace(0,freqmax,numfft);
  fftraw = fftraw/blocksize*2;            % scale FFT
  fftraw(1) = fftraw(1)/2;                % correct DC value
  plot(f,abs(fftraw(1:numfft)));
  xlabel('frequency (Hz)');
  ylabel('ampl (orig)');

  % filtered signal in frequency domain

  subplot(4,1,3);
  fftfiltered = fftfiltered/blocksize*2;  % scale FFT
  fftfiltered(1) = fftfiltered(1)/2;      % correct DC value
  plot(f,abs(fftfiltered(1:numfft)));
  xlabel('frequency (Hz)');
  ylabel('ampl (filt)');

  % return filtered signal back to time domain

  subplot(4,1,4);
  plot(t,real(yfilt));
  xlabel('time (sec)');
  ylabel('ampl (filt)');
end