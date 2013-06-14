function [A,B] = cheby2Setup(n, r, Wst)
%
%       function [] = cheby2Setup(n, r, Wst)
%            - returns the A and B coefficients for a digital low pass 
%            - filter in a formatted TXT file to be used by the c++ script
%
%
%       input:
%           n - filter order 
%           r - stopband ripple is r decibels down
%           Wst - stopband edge frequency (0-1.0)

clc;

if nargin == 1, fprintf('\n Error: Specify filter order (interger value from 0 to 500). \n'); end;
if nargin == 2, fprintf('\n Error: Specify attenuation of stopband, in decibals \n'); end;
if nargin == 3, Wst = 0.05; end;    


%Specify your location to create the TXT here.
cd('C:\Users\Brandon\Dropbox\Github\Quadcopter\Data_Fourier Analysis');

%This is a built in matlab function that does the magic.
[A,B] = cheby2(n,r,Wst);

myString = strcat('Order = ',num2str(n),',',' r = ',num2str(r),',',' Wst = ',num2str(Wst)); 

%creates the file for r/w, removes existing contents
myTXT_ID = fopen('ChebyCoeffs.txt','w+');   

%print the data to the file
fprintf(myTXT_ID, '%s\n', 'Coefficents and filter specs:'); 
fprintf(myTXT_ID, '%s\n\n', myString);

for i = 1:(n+1)
    fprintf(myTXT_ID, '%12.7f', A(i));
    fprintf(myTXT_ID, '%12.7f\n', B(i));
end

%close the file for use by other programs
fclose(myTXT_ID);
end

