function [ in, state, out, sampleTimes ] = processCtrlData( logData )
%LOADCTRLDATA Summary of this function goes here
%   Detailed explanation goes here

% define fixed parameters
Fs = 1000;
Ts = 1/Fs;

front = 31;
back = 45;
botRadius = 0.079;
wheelRadius = 0.03;
motor2Wheel = 1.0;

if isfield(logData, 'ctrl_physical')
    frontCol = strcmp(logData.ctrl_physical.names, 'front_angle');
    backCol = strcmp(logData.ctrl_physical.names, 'back_angle');
    botRadiusCol = strcmp(logData.ctrl_physical.names, 'bot_radius');
    wheelRadiusCol = strcmp(logData.ctrl_physical.names, 'wheel_radius');
    
    front = logData.ctrl_physical.data(1,frontCol);
    back = logData.ctrl_physical.data(1,backCol);
    botRadius = logData.ctrl_physical.data(1,botRadiusCol);
    wheelRadius = logData.ctrl_physical.data(1,wheelRadiusCol);
else
    disp('No ctrl_physical in logfile, using old defaults');
end

if isfield(logData, 'ctrl_drive_train')
    motor2WheelCol = strcmp(logData.ctrl_drive_train.names, 'motor2wheel');
    
    motor2Wheel = logData.ctrl_drive_train.data(1,motor2WheelCol);
else
    disp('No ctrl_drive_train in logfile, using old defaults');
end

front = front*pi/180;
back = back*pi/180;

% friction = [1.35,1.02];
friction = [1,1];
theta = [front, pi-front, pi+back, 2*pi-back];
D = [-sin(theta')*friction(1), cos(theta')*friction(2), ones(4,1)*botRadius];
Dx = pinv(D);
% DW = [-cos(theta'), sin(theta'), zeros(4,1)];
% DWx = pinv(DW);
v_mot_none = null(Dx); % paper description f_k

% extract start and end time
tStart = roundn(logData.sensors.data(1,1)+0.1, -1);
tEnd = roundn(logData.sensors.data(end-1,1)-0.1, -1);

sampleTimes = (tStart:Ts:tEnd)';
numSamples = length(sampleTimes);

[in.pos.gyr.local, in.vel.gyr.local] = procGyr();
[in.pos.acc.local, in.vel.acc.local, in.acc.acc.local, in.jerk.acc.local] = procAcc();
[in.mot.vel, in.enc.vel, in.slip.enc, in.vel.enc.local, in.pos.enc.local] = procMotorEncIn();
[in.pos.vis.global, in.pos.vis.local, in.vel.vis.local, in.acc.vis.local, in.pos.vis.dt, in.mot.vis.vel] = procVision();
[out.mot.vel, out.enc.vel, out.slip.enc, out.vel.enc.local] = procMotorEncOut();
[out.mot.vol] = procMotorVol();
[out.mot.cur] = procMotorCurOut();
[in.mot.cur] = procMotorCur();
[state.pos.global, state.vel.global, state.acc.global, state.pos.local, state.vel.local, state.acc.local, state.vel.localAcc] = procState();
[in.bat.vol, in.bat.cur] = procBat();
[in.kicker.vol, in.kicker.cur, in.kicker.cnt, in.kicker.flags] = procKicker();
[in.barrier.on, in.barrier.off, in.barrier.irq] = procBarrier();
[in.dribbler.speed, in.dribbler.voltage, in.dribbler.temp, in.dribbler.current] = procDribbler();

function tCor = timeRolloverComp(tIn)
    cor = cumsum([0; diff(tIn) < -2147483648])*4294967296;
    tCor = tIn + cor;
end

function [ orientation, velocity ] = procGyr()
    firstCol = find(strcmp(logData.sensors.names, 'gyr_updated'));
    
    gyrVelData = logData.sensors.data(:,firstCol+1:firstCol+4);
    velocity = zeros(numSamples, 3);
    orientation = zeros(numSamples, 3);
    
    gyrVelData(:,1) = timeRolloverComp(gyrVelData(:,1));
    
    [~, ia] = unique(gyrVelData(:,1));
    gyrVelData = gyrVelData(ia, :);
    gyrVelData(:,1) = gyrVelData(:,1)*1e-6;
    
	if size(gyrVelData,1) < 2
        return;
	end
    
    velocity(:,1) = interp1(gyrVelData(:,1), gyrVelData(:,2), sampleTimes);
    velocity(:,2) = interp1(gyrVelData(:,1), gyrVelData(:,3), sampleTimes);
    velocity(:,3) = interp1(gyrVelData(:,1), gyrVelData(:,4), sampleTimes);
    
    orientation(:,1) = cumtrapz(sampleTimes, velocity(:,1));
    orientation(:,2) = cumtrapz(sampleTimes, velocity(:,2));
    orientation(:,3) = cumtrapz(sampleTimes, velocity(:,3));
end

function [ vol, cur ] = procBat()
    firstCol = find(strcmp(logData.sensors.names, 'supply_bat'));
    
    batData = logData.sensors.data(:,[1 firstCol:firstCol+1]);
    
    vol = zeros(numSamples, 1);
    cur = zeros(numSamples, 1);
    
    batData(:,1) = timeRolloverComp(batData(:,1));
    
    [~, ia] = unique(batData(:,1));
    batData = batData(ia, :);
    
	if size(batData,1) < 2
        return;
	end
    
    vol(:,1) = interp1(batData(:,1), batData(:,2), sampleTimes);
    cur(:,1) = interp1(batData(:,1), batData(:,3), sampleTimes);
end

function [posGlobal, velGlobal, accGlobal, posLocal, velLocal, accLocal, accVelLocal] = procState()
    time = logData.ctrl_state.data(:,1);
    posData = logData.ctrl_state.data(:,2:4);
    velData = logData.ctrl_state.data(:,5:7);
    accData = logData.ctrl_state.data(:,8:10);
    
    [~, ia] = unique(time);	% find unqiue entries
    time = time(ia, :);
    posData = posData(ia, :);	% delete duplicates
    velData = velData(ia, :);	% delete duplicates
    accData = accData(ia, :);	% delete duplicates

    velLocalData = velData;
    accLocalData = accData;
    
    % Multiturn angle correction for vision data
	correctionTurns = 0;
	newOrient = zeros(length(posData), 1);
	newOrient(1) = posData(1, 3);
	for i = 2:length(posData)
		angleChange = posData(i-1,3)-posData(i,3);
		if angleChange > 1.5*pi
			correctionTurns = correctionTurns + 1;
		end
		if angleChange < -1.5*pi
			correctionTurns = correctionTurns - 1;
		end
		
		newOrient(i) = posData(i,3)+correctionTurns*2*pi;
	end
	posData(:,3) = newOrient;
	
	% Align positions with local frame
	angle = pi/2-posData(1,3);
	rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];

	posLocalData = posData - repmat(posData(1,1:3), length(posData), 1); % remove offset
	posLocalData(:,1:2) = (rotMat*(posLocalData(:,1:2)'))';
    
    posGlobal = zeros(numSamples, 3);
    velGlobal = zeros(numSamples, 4);
    accGlobal = zeros(numSamples, 4);
    
    posGlobal(:,1) = interp1(time, posData(:,1), sampleTimes);
    posGlobal(:,2) = interp1(time, posData(:,2), sampleTimes);
    posGlobal(:,3) = interp1(time, posData(:,3), sampleTimes);
    
    velGlobal(:,1) = interp1(time, velData(:,1), sampleTimes);
    velGlobal(:,2) = interp1(time, velData(:,2), sampleTimes);
    velGlobal(:,3) = interp1(time, velData(:,3), sampleTimes);
    
    velGlobal(:,4) = sqrt(velGlobal(:,1).^2+velGlobal(:,2).^2);
    
    accGlobal(:,1) = interp1(time, accData(:,1), sampleTimes);
    accGlobal(:,2) = interp1(time, accData(:,2), sampleTimes);
    accGlobal(:,3) = interp1(time, accData(:,3), sampleTimes);
    
    accGlobal(:,4) = sqrt(accGlobal(:,1).^2+accGlobal(:,2).^2);

    posLocal = zeros(size(posGlobal));
    velLocal = velGlobal;
    accLocal = accGlobal;

    posLocal(:,1) = interp1(time, posLocalData(:,1), sampleTimes);
    posLocal(:,2) = interp1(time, posLocalData(:,2), sampleTimes);
    posLocal(:,3) = interp1(time, posLocalData(:,3), sampleTimes);
    
    velLocal(:,1) = interp1(time, velLocalData(:,1), sampleTimes);
    velLocal(:,2) = interp1(time, velLocalData(:,2), sampleTimes);
    velLocal(:,3) = interp1(time, velLocalData(:,3), sampleTimes);

    accLocal(:,1) = interp1(time, accLocalData(:,1), sampleTimes);
    accLocal(:,2) = interp1(time, accLocalData(:,2), sampleTimes);
    accLocal(:,3) = interp1(time, accLocalData(:,3), sampleTimes);

    for i = 1:numSamples
        angle = pi/2-posGlobal(i,3);
		rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
		velLocal(i,1:2) = (rotMat*(velLocal(i,1:2)'))';
        accLocal(i,1:2) = (rotMat*(accLocal(i,1:2)'))';
    end
    
    accVelLocal = accLocal;
    
    accVelLocal(:,1) = cumtrapz(sampleTimes, accLocal(:,1));
    accVelLocal(:,2) = cumtrapz(sampleTimes, accLocal(:,2));
    accVelLocal(:,3) = cumtrapz(sampleTimes, accLocal(:,3));
end

function [position, velocity, accel, jerk] = procAcc()
    firstCol = find(strcmp(logData.sensors.names, 'acc_updated'));
    
    accData = logData.sensors.data(:,firstCol+1:firstCol+4);
    position = zeros(numSamples, 3);
    velocity = zeros(numSamples, 4);
    accel = zeros(numSamples, 3);
    jerk = zeros(numSamples, 3);
    
    accData(:,1) = timeRolloverComp(accData(:,1));
    
    [~, ia] = unique(accData(:,1));
    accData = accData(ia, :);
    accData(:,1) = accData(:,1)*1e-6;
    
	if size(accData,1) < 2
        return;
	end

    accel(:,1) = interp1(accData(:,1), accData(:,2), sampleTimes);
    accel(:,2) = interp1(accData(:,1), accData(:,3), sampleTimes);
    accel(:,3) = interp1(accData(:,1), accData(:,4), sampleTimes);
    
    jerk(:,1) = [0; diff(accel(:,1))/Ts];
    jerk(:,2) = [0; diff(accel(:,2))/Ts];
    jerk(:,3) = [0; diff(accel(:,3))/Ts];
	
% 	offsets = mean(accel(1:1000,:));
% 	accel(:,1) = accel(:,1) - offsets(1);
% 	accel(:,2) = accel(:,2) - offsets(2);
% 	accel(:,3) = accel(:,3) - offsets(3);
    
%     for i = 1:numSamples
% 		angle = in.pos.gyr.local(i,3);
% 		rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
% 		accel(i,1:2) = (rotMat*(accel(i,1:2)'))';
%     end
    
    velocity(:,1) = cumtrapz(sampleTimes, accel(:,1));
    velocity(:,2) = cumtrapz(sampleTimes, accel(:,2));
    velocity(:,3) = cumtrapz(sampleTimes, accel(:,3));
    
    velocity(:,4) = sqrt(velocity(:,1).^2+velocity(:,2).^2);
    
    position(:,1) = cumtrapz(sampleTimes, velocity(:,1));
    position(:,2) = cumtrapz(sampleTimes, velocity(:,2));
    position(:,3) = cumtrapz(sampleTimes, velocity(:,3));
	
% 	accel(:,1) = smooth(accel(:,1), 100);
% 	accel(:,2) = smooth(accel(:,2), 100);
% 	accel(:,3) = smooth(accel(:,3), 100);
end

function [motVel, encVel, encSlip, localVel, posLocal] = procMotorEncIn()
    firstCol = find(strcmp(logData.sensors.names, 'vel_t'));
    
    motorVelCurData = logData.sensors.data(:,firstCol:firstCol+4);
    motVel = zeros(numSamples, 4);
    
    motorVelCurData(:,1) = timeRolloverComp(motorVelCurData(:,1));
    
    motorVelCurData(:,1) = motorVelCurData(:,1)*1e-6;
    [~, ia] = unique(motorVelCurData(:,1));

    for m = 1:4
        mOrig = [motorVelCurData(ia, 1) motorVelCurData(ia, 1+m)];
        motVel(:,m) = interp1(mOrig(:,1), mOrig(:,2), sampleTimes);
    end
    
    encVel = motVel*motor2Wheel*wheelRadius;
    
    encSlip = zeros(numSamples,1);
    for i = 1:numSamples
        encSlip(i) = dot(encVel(i,:), v_mot_none);
    end
    
    localVel = zeros(numSamples, 4);
    for i = 1:numSamples
        localVel(i,1:3) = Dx*encVel(i,:)';
    end
    
%     localVel(:,1) = localVel(:,1)*0.7 - sign(localVel(:,1)).*localVel(:,1).^2*0.03;
%     localVel(:,2) = localVel(:,2)*0.85 - sign(localVel(:,2)).*localVel(:,2).^2*0.02;
    
    localVel(:,4) = sqrt(localVel(:,1).^2+localVel(:,2).^2);
    
    posLocal = zeros(numSamples, 3);
	posLocal(:,3) = cumtrapz(sampleTimes, localVel(:,3));
	
	fixedVel = zeros(numSamples, 2);
	for i = 1:numSamples
		angle = in.pos.gyr.local(i,3);
		rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
		fixedVel(i,:) = (rotMat*(localVel(i,1:2)'))';
	end
	
    posLocal(:,1) = cumtrapz(sampleTimes, fixedVel(:,1));
    posLocal(:,2) = cumtrapz(sampleTimes, fixedVel(:,2));
end

function [motVelSet, encVelSet, encSlip, xyw] = procMotorEncOut()
    firstCol = find(strcmp(logData.ctrl_output.names, 'vel_m1'));
    
    motorVelSetData = logData.ctrl_output.data(:,firstCol:firstCol+3);
    motVelSet = zeros(numSamples, 4);

    if size(logData.ctrl_output.data,1) > 0
      for m = 1:4
          motVelSet(:, m) = interp1(logData.ctrl_output.data(:,1), motorVelSetData(:,m), sampleTimes);
      end
    end
    
    encVelSet = motVelSet*1/(2*pi)*motor2Wheel*2*wheelRadius*pi;

    encSlip = zeros(numSamples,1);
    for i = 1:numSamples
        encSlip(i) = dot(encVelSet(i,:), v_mot_none);
    end
    
    xyw = zeros(numSamples, 4);
    for i = 1:numSamples
        xyw(i,1:3) = Dx*encVelSet(i,:)';
    end
    
    xyw(:,4) = sqrt(xyw(:,1).^2+xyw(:,2).^2);
end

function [motCurSet] = procMotorCurOut()
    firstCol = find(strcmp(logData.ctrl_output.names, 'torque_m1'));
    
    motorTorqueSetData = logData.ctrl_output.data(:,firstCol:firstCol+3);
    motTorqueSet = zeros(numSamples, 4);

    if size(logData.ctrl_output.data,1) > 0
      for m = 1:4
          motTorqueSet(:, m) = interp1(logData.ctrl_output.data(:,1), motorTorqueSetData(:,m), sampleTimes);
      end
    end
    
    km = 0.033168;
    if isfield(logData, 'ctrl_drive_train') && size(logData.ctrl_drive_train.data, 1) > 0
        km = logData.ctrl_drive_train.data(1,4);
        disp('Using Km from logfile');
    else
    end
    
    motCurSet = motTorqueSet * 1/km;
end

function [voltage] = procMotorVol()
    firstCol = find(strcmp(logData.sensors.names, 'vol_t'));
    
    motorVolSetData = logData.sensors.data(:,firstCol:firstCol+4);
    voltage = zeros(numSamples, 4);
    
    motorVolSetData(:,1) = timeRolloverComp(motorVolSetData(:,1));

    [~, ia] = unique(motorVolSetData(:,1));
    
    for m  = 1:4
        mOrig = [motorVolSetData(ia, 1) motorVolSetData(ia, 1+m)];
        if length(mOrig) < 10
            continue;
        end
        mOrig(:,1) = mOrig(:,1)*1e-6;
        voltage(:,m) = interp1(mOrig(:,1), mOrig(:,2), sampleTimes);
    end
end

function [current] = procMotorCur()
    firstCol = find(strcmp(logData.sensors.names, 'cur_t'));
    
    current = zeros(numSamples, 4);
    motorCurSetData = logData.sensors.data(:,firstCol:firstCol+4);
    
    if length(motorCurSetData) < 10
        return;
    end
    
    motorCurSetData(:,1) = timeRolloverComp(motorCurSetData(:,1));

    [~, ia] = unique(motorCurSetData(:,1));
    
    for m  = 1:4
        mOrig = [motorCurSetData(ia, 1) motorCurSetData(ia, 1+m)];
        if length(mOrig) < 10
            continue;
        end
        mOrig(:,1) = mOrig(:,1)*1e-6;
        current(:,m) = interp1(mOrig(:,1), mOrig(:,2), sampleTimes);
    end
end

function [posGlobal, posLocal, velLocal, accLocal, dT, velMot] = procVision()
    % col 30 is pos_updated
    firstCol = find(strcmp(logData.sensors.names, 'pos_t'));
    
    visionPosData = logData.sensors.data(:,firstCol:firstCol+4);	% input
    posGlobal = zeros(numSamples, 3);	% output
    posLocal = zeros(numSamples, 3);	% output
    velLocal = zeros(numSamples, 4);	% output
	accLocal = zeros(numSamples, 4);	% output
    dT = zeros(numSamples, 1);          % output
    velMot = zeros(numSamples, 4);          % output
    visionPosData(:,1) = timeRolloverComp(visionPosData(:,1));
    [~, ia] = unique(visionPosData(:,1));	% find unqiue entries
    visionPosData = visionPosData(ia, :);	% delete duplicates
	visionPosData(1,:) = []; % delete first row
    visionPosData(:,1:2) = visionPosData(:,1:2)*1e-6;	% scale time to [s]
    visionPosData(:,1) = visionPosData(:,1)-visionPosData(:,2); % subtract BS delay
    visionPosLocalData = zeros(size(visionPosData));
    
	if length(visionPosData) < 10
        return;
	end
    
    dTData = [0; diff(visionPosData(:,1))];
    
    visionPosData(:,2) = []; % remove delay column
	
	% Multiturn angle correction for vision data
	correctionTurns = 0;
	newOrient = zeros(length(visionPosData), 1);
	newOrient(1) = visionPosData(1, 4);
	for i = 2:length(visionPosData)
		angleChange = visionPosData(i-1,4)-visionPosData(i,4);
		if angleChange > 1.5*pi
			correctionTurns = correctionTurns + 1;
		end
		if angleChange < -1.5*pi
			correctionTurns = correctionTurns - 1;
		end
		
		newOrient(i) = visionPosData(i,4)+correctionTurns*2*pi;
	end
	visionPosData(:,4) = newOrient;
    
    visionPosData(:,4) = smooth(visionPosData(:,4), 10);
	
	% Align positions with local frame
	angle = pi/2-visionPosData(1,4);
	rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];

	visionPosLocalData(:,1) = visionPosData(:,1);
	visionPosLocalData(:,2:4) = visionPosData(:,2:4) - repmat(visionPosData(1,2:4), length(visionPosData), 1);
	visionPosLocalData(:,2:3) = (rotMat*(visionPosLocalData(:,2:3)'))';
    
    for i = 1:3
        posGlobal(:,i) = interp1(visionPosData(:,1), visionPosData(:,i+1), sampleTimes, 'linear', 'extrap');
        posLocal(:,i) = interp1(visionPosLocalData(:,1), visionPosLocalData(:,i+1), sampleTimes, 'linear', 'extrap');
        velLocal(:,i) = [0; diff(posGlobal(:,i))/Ts];
%         velLocal(:,i) = smooth(velLocal(:,i), 5);
        accLocal(:,i) = [0; diff(velLocal(:,i))/Ts];
        accLocal(:,i) = smooth(accLocal(:,i), 100);
    end
    
    dT = interp1(visionPosData(:,1), dTData, sampleTimes, 'previous', 'extrap');
    
    velLocal(:,4) = sqrt(velLocal(:,1).^2+velLocal(:,2).^2);
	accLocal(:,4) = sqrt(accLocal(:,1).^2+accLocal(:,2).^2);
    
    for i = 1:numSamples
        angle = pi/2-posGlobal(i,3);
		rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
		velLocal(i,1:2) = (rotMat*(velLocal(i,1:2)'))';
        accLocal(i,1:2) = (rotMat*(accLocal(i,1:2)'))';
    end
    
    velMot = (D*velLocal(:,1:3)')';
    velMot = velMot.*(1/motor2Wheel*1/wheelRadius);
end

function [vol, chg, cnt, flags] = procKicker()
    firstCol = find(strcmp(logData.sensors.names, 'kicker_level'));
    
    kickerData = logData.sensors.data(:,[1 firstCol:firstCol+3]);
    
    kickerData(:,1) = timeRolloverComp(kickerData(:,1));

    [~, ia] = unique(kickerData(:,1));
    kickerData = kickerData(ia, :);	% delete duplicates

    vol = interp1(kickerData(:,1), kickerData(:,2), sampleTimes);
    chg = interp1(kickerData(:,1), kickerData(:,3), sampleTimes);
    cnt = interp1(kickerData(:,1), kickerData(:,4), sampleTimes);
    flags = interp1(kickerData(:,1), kickerData(:,5), sampleTimes);
end

function [vOn, vOff, interrupted] = procBarrier()
    firstCol = find(strcmp(logData.sensors.names, 'ir_on'));
    
    irData = logData.sensors.data(:,[1 firstCol:firstCol+2]);
    
    irData(:,1) = timeRolloverComp(irData(:,1));

    [~, ia] = unique(irData(:,1));
    irData = irData(ia, :);	% delete duplicates

    vOn = interp1(irData(:,1), irData(:,2), sampleTimes);
    vOff = interp1(irData(:,1), irData(:,3), sampleTimes);
    interrupted = interp1(irData(:,1), irData(:,4), sampleTimes);
end

function [speed, voltage, temp, current] = procDribbler()
    firstCol = find(strcmp(logData.sensors.names, 'dribbler_speed'));
    
    dribData = logData.sensors.data(:,[1 firstCol:firstCol+4]);
    
    dribData(:,1) = timeRolloverComp(dribData(:,1));

    [~, ia] = unique(dribData(:,1));
    dribData = dribData(ia, :);	% delete duplicates

    speed = interp1(dribData(:,1), dribData(:,2), sampleTimes);
    temp = interp1(dribData(:,1), dribData(:,3), sampleTimes);
    voltage = interp1(dribData(:,1), dribData(:,4), sampleTimes);
    
    if any(strcmp(logData.sensors.names, 'dribbler_current'))
        current = interp1(dribData(:,1), dribData(:,5), sampleTimes);
    else
        current = zeros(numSamples, 1);
    end
    
end

end