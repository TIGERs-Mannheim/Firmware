function [ skill ] = processSkillData( logData, sampleTimes )
%PROCESSDEBUGDATA Summary of this function goes here
%   Detailed explanation goes here

numSamples = length(sampleTimes);
[skill.drive.pos.global, skill.drive.vel.local] = procSetpoints();
[skill.drive.modeXY, skill.drive.modeW] = procMode();
[skill.lim.velXY, skill.lim.velW, skill.lim.accXY, skill.lim.accW] = procDriveLimits();
[skill.dribbler.speed, skill.dribbler.maxForce, skill.dribbler.voltage, skill.dribbler.mode] = procDribbler();
[skill.kicker.mode, skill.kicker.device, skill.kicker.speed] = procKicker();

function tCor = timeRolloverComp(tIn)
    cor = cumsum([0; diff(tIn) < -2147483648])*4294967296;
    tCor = tIn + cor;
end

function [velMaxXY, velMaxW, accMaxXY, accMaxW] = procDriveLimits()
	firstCol = find(strcmp(logData.skill_output.names, 'vel_max_xy'));

	limData = logData.skill_output.data(:,[1 firstCol:firstCol+3]);

	velMaxXY = zeros(numSamples, 1);
    velMaxW = zeros(numSamples, 1);
    accMaxXY = zeros(numSamples, 1);
    accMaxW = zeros(numSamples, 1);
    
    if length(limData) < 10
        return;
    end

    limData(:,1) = timeRolloverComp(limData(:,1));
	[~, ia] = unique(limData(:,1));	% find unqiue entries
	limData = limData(ia, :);	% delete duplicates

    velMaxXY = interp1(limData(:,1), limData(:,2), sampleTimes);
    accMaxXY = interp1(limData(:,1), limData(:,3), sampleTimes);
    velMaxW = interp1(limData(:,1), limData(:,4), sampleTimes);
    accMaxW = interp1(limData(:,1), limData(:,5), sampleTimes);
end

function [setPosGlobal, setVelLocal] = procSetpoints()
	firstCol = find(strcmp(logData.skill_output.names, 'pos_x'));

	setData = logData.skill_output.data(:,[1 firstCol:firstCol+5]);

	setPosGlobal = zeros(numSamples, 3);
	setVelLocal = zeros(numSamples, 3);
    
    if length(setData) < 10
        return;
    end

    setData(:,1) = timeRolloverComp(setData(:,1));
	[~, ia] = unique(setData(:,1));	% find unqiue entries
	setData = setData(ia, :);	% delete duplicates

	for i = 1:3
		setPosGlobal(:,i) = interp1(setData(:,1), setData(:,i+1), sampleTimes);
		setVelLocal(:,i) = interp1(setData(:,1), setData(:,i+4), sampleTimes);
	end
end

function [modeXY, modeW] = procMode()
	firstCol = find(strcmp(logData.skill_output.names, 'mode_xy'));

	ctrlModeData = logData.skill_output.data(:,[1 firstCol:firstCol+1]);

	modeXY = zeros(numSamples, 1);
    modeW = zeros(numSamples, 1);

    if size(ctrlModeData,1) < 10
        return;
    end

    ctrlModeData(:,1) = timeRolloverComp(ctrlModeData(:,1));
	[~, ia] = unique(ctrlModeData(:,1));	% find unqiue entries
	ctrlModeData = ctrlModeData(ia, :);	% delete duplicates

	modeXY(:,1) = interp1(ctrlModeData(:,1), ctrlModeData(:,2), sampleTimes);
    modeW(:,1) = interp1(ctrlModeData(:,1), ctrlModeData(:,3), sampleTimes);
end

function [speed, maxForce, voltage, mode] = procDribbler()
    firstCol = find(strcmp(logData.skill_output.names, 'dribbler_vel'));
    
    dribData = logData.skill_output.data(:,[1 firstCol:firstCol+3]);

    dribData(:,1) = timeRolloverComp(dribData(:,1));
    [~, ia] = unique(dribData(:,1));
    dribData = dribData(ia, :);	% delete duplicates

    speed = interp1(dribData(:,1), dribData(:,2), sampleTimes);
    maxForce = interp1(dribData(:,1), dribData(:,3), sampleTimes);
    voltage = interp1(dribData(:,1), dribData(:,4), sampleTimes);
    mode = interp1(dribData(:,1), dribData(:,5), sampleTimes);
end

function [mode, device, speed] = procKicker()
    firstCol = find(strcmp(logData.skill_output.names, 'kicker_mode'));
    
    kickData = logData.skill_output.data(:,[1 firstCol:firstCol+2]);

    kickData(:,1) = timeRolloverComp(kickData(:,1));
    [~, ia] = unique(kickData(:,1));
    kickData = kickData(ia, :);	% delete duplicates

    mode = interp1(kickData(:,1), kickData(:,2), sampleTimes);
    device = interp1(kickData(:,1), kickData(:,3), sampleTimes);
    speed = interp1(kickData(:,1), kickData(:,4), sampleTimes);
end

end
