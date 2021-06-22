function [ ref ] = processRefData( logData, sampleTimes )
%PROCESSDEBUGDATA Summary of this function goes here
%   Detailed explanation goes here

numSamples = length(sampleTimes);
[ref.pos.global, ref.vel.global, ref.acc.global, ...
    ref.vel.local, ref.acc.local] = procTraj();

function tCor = timeRolloverComp(tIn)
    cor = cumsum([0; diff(tIn) < -2147483648])*4294967296;
    tCor = tIn + cor;
end

function [pos, velGlobal, accGlobal, velLocal, accLocal] = procTraj()
	firstCol = find(strcmp(logData.ctrl_ref.names, 'traj_pos_x'));

	trajData = logData.ctrl_ref.data(:,[1 firstCol:firstCol+8]);

    pos = zeros(numSamples, 3);
    velGlobal = zeros(numSamples, 3);
	accGlobal = zeros(numSamples, 3);
    velLocal = zeros(numSamples, 3);
	accLocal = zeros(numSamples, 3);
    
    if length(trajData) < 10
        return;
    end
    
    trajData(:,1) = timeRolloverComp(trajData(:,1));

	[~, ia] = unique(trajData(:,1));	% find unqiue entries
	trajData = trajData(ia, :);	% delete duplicates

	for i = 1:3
        pos(:,i) = interp1(trajData(:,1), trajData(:,i+1), sampleTimes);
        velLocal(:,i) = interp1(trajData(:,1), trajData(:,i+4), sampleTimes);
		accLocal(:,i) = interp1(trajData(:,1), trajData(:,i+7), sampleTimes);
	end
    
    for i = 1:numSamples
        angle = -pi/2+pos(i,3);
		rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
        
		velGlobal(i,1:2) = (rotMat*(velLocal(i,1:2)'))';
        accGlobal(i,1:2) = (rotMat*(accLocal(i,1:2)'))';
    end
    
    velGlobal(:,3) = velLocal(:,3);
    accGlobal(:,3) = accLocal(:,3);
end

end
