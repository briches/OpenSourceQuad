function [f,ffty] = timefreq(y,dt,meanrem,window,linetype,plottype)
%
% function [f,ffty] = timefreq(y,dt,meanrem,window,linetype)
%
% - function file to perform a Fast Fourier Transform of a data vector
% - plots both the time and frequency domain traces
%
% input:       y - data vector
%             dt - spacing of signal in time domain
%        meanrem - mean removal       (0:no,    1:yes)
%         window - window type        (0:none,  1:Hanning, 2:Hamming)
%       linetype - spectral line type (0:stem,  1:line)
%       plottype - freq plot type     (0:none   1:linear 2:semilog, 3:loglog)
%
% output:      f - frequency vector
%           ffty - scaled Fourier transform of signal
%
%        plot of signal in time and frequency domain

% set some defaults

f = 0;
ffty = 0;

if nargin == 0, fprintf('\n  Error: specify an input data vector!\n\n'); return; end
if nargin == 1, dt=1; meanrem=0; window=0; linetype=0; plottype=1; end
if nargin == 2,       meanrem=0; window=0; linetype=0; plottype=1; end
if nargin == 3,                  window=0; linetype=0; plottype=1; end
if nargin == 4,                            linetype=0; plottype=1; end
if nargin == 5,                                        plottype=1; end

% Fourier transform calculation

blocksize = length(y);
if (blocksize/2 - floor(blocksize/2)) ~= 0 % check to see if blocksize is even
  fprintf('\n NOTE: reduced data block length by one to make an even number\n\n');
  y = y(1:end-1);
  blocksize = blocksize - 1;
end

if size(y,1) ~= 1; y = y'; end             % convert to a row vector if necessary

if meanrem == 1                            % mean removal
  y = y - mean(y);
end

if window == 1                             % window application and scaling
  y = y .* hanning(blocksize)' * 2.0;
elseif window == 2
  y = y .* hamming(blocksize)' * 1/0.54;
end                                        
                                           
ffty = fft(y)/blocksize*2;                 % perform FFT and scale
ffty(1) = ffty(1)/2;                       % correct DC value
numfft = length(ffty)/2 + 1;               % # of positive freq comps
ffty = ffty(1:numfft);                     % only save the positive freq comps
f = linspace(0,1/dt/2,numfft);             % frequency vector

if plottype > 0                            % only plot if called for
  clf;  % clear figures
  subplot(2,1,1);

  t = 0:dt:(blocksize-1)*dt;               % create time vector for plotting
  plot(t,y);                               % time domain plot
  xlabel('time (sec)');
  ylabel('amplitude');
  title('TIMEFREQ: time and frequency domain plots of a signal');

  % for the FFT plot, only plot length(ffty)/2 + 1 points (i.e. one sided)
  % and scale x-axis as frequency...

  subplot(2,1,2);

  if plottype == 1
    if linetype == 1
      plot(f,abs(ffty));
    else
      stem(f,abs(ffty));
    end
  elseif plottype == 2
    if linetype == 1
      semilogy(f,abs(ffty));
    else
      semilog(f,abs(ffty),'o');
    end
  elseif plottype == 3
    if linetype == 1
      loglog(f(2:numfft),abs(ffty(2:numfft)));
    else
      loglog(f(2:numfft),abs(ffty(2:numfft)),'o');
    end
  end
  xlabel('frequency (Hz)');
  ylabel('amplitude');
end

% if nargout == 0
%   f = [];
%   ffty = [];
% end