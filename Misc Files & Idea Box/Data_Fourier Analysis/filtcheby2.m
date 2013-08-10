function [filtdata] = filtcheby2(rawdata,fsamp,fcut,hilo,plotswitch)
%
% function filt(rawdata,fsamp,fcut,hilo,plotswitch)
%
% - performs low or high-pass filtering based on a Chebychev filter type 
% - returns the filtered data
%
% input:   rawdata - input raw data vector
%            fsamp - sampling rate of raw data
%             fcut - filter cut-off frequency
%             hilo - filter type (0:low-pass, 1:high-pass) - defaults to low-pass
%       plotswitch - plot data? (0:no, 1:yes) - defaults to yes
%
% output: filtdata - filtered data set
%         plot of raw and filtered data

if nargin == 0, fprintf('\n  Error: specify input vector, sampling frequency and cutoff frequency\n\n'); end
if nargin == 1, fprintf('\n  Error: specify sampling frequency and cutoff frequency\n\n'); end
if nargin == 2, fprintf('\n  Error: specify cutoff frequency\n\n'); end
if nargin == 3, hilo=0; plotswitch=1; end
if nargin == 4,         plotswitch=1; end

if hilo == 1 
	highswitch = 'high'; 
else highswitch = 'low'; 
end 

[b,a] = cheby2(10,40,fcut/(fsamp/2),highswitch);
filtdata = filtfilt(b,a,rawdata);
if plotswitch == 1
  dt = 1/fsamp;
  tvect = (0:dt:dt*(length(rawdata)-1));
  plot(tvect,rawdata,'b',tvect,filtdata,'r')
  legend('raw','filtered')
end