function [s] = quadDataAnalyze()
    clf; clc;
    % Works off the data in the D: drive
    cd 'D:\';

    % List available files, pick the file
    s = dir('*.txt');
    file_list = {s.name};
    selection = menu('Select a file',file_list);

    % Open the selected file
    if length(file_list)==1
        filename = s.name;
        fid = fopen(filename,'r');
    else 
        filename = s.name(selection);
        fid = fopen(filename,'r');
    end

    % Find the first line of .csv format
    sline = getLine(fid);
    fclose(fid);

    % Read the csv data
    csvdata = csvread(filename,sline,0);
    time = csvdata(:,1);
    dt = (time(2)-time(1))/1000000;
    filt = csvdata(:,2);
    nfilt = csvdata(:,3);

    % Plot that there data
    figure(9); hold on; grid on;
    plot(time, filt, 'b');
    plot(time, nfilt,'r');
    legend('Filtered','Unfiltered','Location','SouthEast');
    hold off;
    
    figure(10);
    [~, ~] = timefreq(filt,dt,1,1,0,1);
    title('Filtered Data');
    figure(11);
    [~, ~] = timefreq(nfilt,dt,1,1,0,1);
    title('Unfiltered data');
    
    % Close 'em
    fclose('all');
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