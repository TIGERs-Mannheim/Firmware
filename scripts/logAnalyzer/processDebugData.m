function [ debug ] = processDebugData( logData, sampleTimes )
%PROCESSDEBUGDATA Summary of this function goes here
%   Detailed explanation goes here

numSamples = length(sampleTimes);
[debug.time.input, debug.time.est, debug.time.skill, ...
    debug.time.ctrl, debug.time.out, debug.time.misc] = procDebugTime();

function tCor = timeRolloverComp(tIn)
    cor = cumsum([0; diff(tIn) < -2147483648])*4294967296;
    tCor = tIn + cor;
end

function [inputTime, estTime, skillTime, ctrlTime, outTime, miscTime] = ...
        procDebugTime()

    inputTime = zeros(numSamples, 1);
	estTime = zeros(numSamples, 1);
	skillTime = zeros(numSamples, 1);
	ctrlTime = zeros(numSamples, 1);
	outTime = zeros(numSamples, 1);
	miscTime = zeros(numSamples, 1);
    
    if ~isfield(logData, 'performance')
        disp('No performance timing data available in logfile');
        return;
    end
    
	firstCol = find(strcmp(logData.performance.names, 'input_time'));
    lastCol = find(strcmp(logData.performance.names, 'misc_time'));

	timeData = logData.performance.data(:,[1 firstCol:lastCol]);
    
    timeData(:,1) = timeRolloverComp(timeData(:,1));
    
	[~, ia] = unique(timeData(:,1));	% find unqiue entries
	timeData = timeData(ia, :);	% delete duplicates
    
    if length(timeData) < 10
        return;
    end

	inputTime(:,1) = interp1(timeData(:,1), timeData(:,2), sampleTimes);
	estTime(:,1) = interp1(timeData(:,1), timeData(:,3), sampleTimes);
	skillTime(:,1) = interp1(timeData(:,1), timeData(:,4), sampleTimes);
	ctrlTime(:,1) = interp1(timeData(:,1), timeData(:,5), sampleTimes);
	outTime(:,1) = interp1(timeData(:,1), timeData(:,6), sampleTimes);
	miscTime(:,1) = interp1(timeData(:,1), timeData(:,7), sampleTimes);
end

end
