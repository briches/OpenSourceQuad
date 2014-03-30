function [s] = quadFiltAnalyze()

% Do this in the kinematics header somewhere
% if (logFile)
%         {
%             logFile->print(micros());
%             logFile->print(',');
%             logFile->print(kinematics.pitch);
%             logFile->print(',');
%             logFile->print(pitchAcc);
%             logFile->print(',');
%             logFile->println(pitchSet);
%         }
%         else
%         {
%             //Serial.println("Err w/in kinematics");
%         }

    clc; close all;
    % Works off the data in the D: drive
    cd 'D:\';

    % List available files, pick the file
    s = dir('*.txt');
    file_list = {s.name};
    

    % Open the selected file
    if length(file_list) < 2
        filename = s.name;
        fid = fopen(filename,'r');
    else 
        selection = menu('Select a file',file_list);
        filename = s.name(selection);
        fid = fopen(filename,'r');
    end

    % Find the first line of .csv format
    sline = getLine(fid);
    fclose(fid);
    filename = lower(filename);

    % Read the csv data
    csvdata = csvread(filename,sline,0);
    time = csvdata(:,1);
    dt = getDt(time);
    filt = csvdata(:,2);
    nfilt = csvdata(:,3);
    set = csvdata(:,4);
    check = (abs(filt - nfilt) < 5);

    % Plot that there data
    figure(9); hold on; grid on;
    plot(time, filt, 'b');
    plot(time, nfilt,'r');
    plot(time, set, 'k');
    plot(time, check, 'c-');
    legend('Gyro+accel','Accel','Setpoint','Location','SouthEast');
    hold off;
    
    figure(10);
    [~, ~] = timefreq(filt,dt,1,1,0,1);
    title('Complementary filtered data');
    figure(11);
    [~, ~] = timefreq(nfilt,dt,1,1,0,1);
    title('Accelerometer data');
    
    % Close 'em
    fclose('all');
    cd 'C:\';
end

function dt = getDt(time)

deltas = zeros(length(time),1);

for n = 2:length(time)
    deltas(n-1) = (time(n)-time(n-1))/1000000;
end

dt = sum(deltas)/length(deltas);
end

function start = getLine(fid)
    lc = 0;
    while(~feof(fid))
        lc = lc + 1;
        line = fgetl(fid);
        if ~isempty(strfind(line,'Runtime data:'))
            start = lc + 3;
            break;
        end
    end
end