clear;
clc;

logData = loadLogFile('trial12.dat');
[in, state, out, sampleTimes] = processCtrlData(logData);
[debug] = processDebugData(logData, sampleTimes);

decimate = 16;

outVelLocal = out.vel.enc.local(:,1:3);
inPosGlobal = in.pos.vis.global; 

visionDelay = 80; % [ms]

% shift vision data to align it with "now" time
inPosGlobal = circshift(inPosGlobal, -visionDelay);

% remove circular shifted data at end => useless
inPosGlobal(end-visionDelay+1:end,:) = [];
outVelLocal(end-visionDelay+1:end,:) = [];
sampleTimes(end-visionDelay+1:end,:) = [];

sampleDtVision = 500; % [ms]
sampleDtVel = 500; % [ms]
numPastSamplesVision = 5;
numPastSamplesVel = 5;

% construct past data columns
posGlobal = repmat(inPosGlobal, 1, numPastSamplesVision+1);
velLocal = repmat(outVelLocal, 1, numPastSamplesVel+1);
for i = 1:numPastSamplesVision
    posGlobal(:,i*3-2:i*3) = circshift(posGlobal(:,i*3-2:i*3), (numPastSamplesVision-i+1)*-sampleDtVision);
end

for i = 1:numPastSamplesVel
    velLocal(:,i*3-2:i*3) = circshift(velLocal(:,i*3-2:i*3), (numPastSamplesVel-i+1)*-sampleDtVel);
end

% remove shifted data at end
posGlobal(end-numPastSamplesVision*sampleDtVision:end,:) = [];
velLocal(end-numPastSamplesVision*sampleDtVision:end,:) = [];
sampleTimes(end-numPastSamplesVision*sampleDtVision:end,:) = [];

numSamples = length(sampleTimes);

% decimate data to speed up neural network
sampleIndex = 1:decimate:numSamples;
posGlobal = posGlobal(sampleIndex,:);
velLocal = velLocal(sampleIndex,:);
sampleTimes = sampleTimes(sampleIndex);

numSamples = length(sampleTimes);

% transform position to local frame
posLocal = posGlobal - repmat(posGlobal(:,end-2:end), 1, numPastSamplesVision+1);

tic;

% rotate position to local frame
for i = 1:numSamples
	angle = pi/2-inPosGlobal(i, 3);
	rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)]';
    
    for j = 1:numPastSamplesVision+1
        posLocal(i, j*3-2:j*3-1) = posLocal(i, j*3-2:j*3-1) * rotMat;
    end
end

nnInput = [posLocal(:, 4:end) velLocal(:, 4:end)];
nnTarget = posLocal(:, 1:3);

toc

nnInput = nnInput';
nnOutput = nnTarget';

net = fitnet(8);
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
net.trainParam.max_fail = 20;
% net.layers{1}.transferFcn = 'logsig';
net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'purelin';
[net,tr] = train(net, nnInput, nnOutput);
% view(net)
outputs = net(nnInput);
errors = gsubtract(outputs, nnOutput);
perf = perform(net, outputs, nnOutput)

% genFunction(net, 'nnFunc', 'MatrixOnly', 'yes', 'ShowLinks', 'no');
% codegen nnFunc.prj;

subplot(2,1,1);
histogram(errors(1:2,:))
title('XY Error Histogram');
axis tight;

subplot(2,1,2);
histogram(errors(3,:))
title('W Error Histogram');
axis tight;
