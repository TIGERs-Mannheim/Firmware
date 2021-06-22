clear;
clc;

logData = loadLogFile('trial12.dat');
[in, state, out, sampleTimes] = processCtrlData(logData);
[debug] = processDebugData(logData, sampleTimes);

outVelLocal = out.vel.enc.local(:,1:3);
inPosGlobal = in.pos.vis.global; 

visionDelay = 80; % [ms]

% shift vision data to align it with "now" time
inPosGlobal = circshift(inPosGlobal, -visionDelay);

% remove circular shifted data at end => useless
inPosGlobal(end-visionDelay+1:end,:) = [];
outVelLocal(end-visionDelay+1:end,:) = [];
sampleTimes(end-visionDelay+1:end,:) = [];

sampleDtVision = 20; % [ms]
numPastSamplesVision = 5;
numPastSamplesVel = 5;

posDiff = [0 0 0; diff(inPosGlobal(1:sampleDtVision:end,:))];
velLocal = outVelLocal(1:sampleDtVision:end,:);
sampleTimes = sampleTimes(1:sampleDtVision:end);
inPosGlobal = inPosGlobal(1:sampleDtVision:end,:);

numSamples = length(sampleTimes);

% rotate position to local frame
posDiffLocal = posDiff;
for i = 1:numSamples
	angle = pi/2-inPosGlobal(i, 3);
	rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)]';
    
    posDiffLocal(i, 1:2) = posDiffLocal(i, 1:2) * rotMat;
end

posDiffAll = repmat(posDiffLocal, 1, numPastSamplesVision+1);
velLocalAll = repmat(velLocal, 1, numPastSamplesVel+1);

for i = 1:numPastSamplesVision
    posDiffAll(:,i*3-2:i*3) = circshift(posDiffAll(:,i*3-2:i*3), -i+visionDelay);
end

for i = 1:numPastSamplesVel
    velLocalAll(:,i*3-2:i*3) = circshift(velLocalAll(:,i*3-2:i*3), -i);
end

% remove shifted data at end
numRemove = max([numPastSamplesVision, numPastSamplesVel]);
posDiffAll(end-numRemove:end,:) = [];
velLocalAll(end-numRemove:end,:) = [];
sampleTimes(end-numRemove:end,:) = [];

numSamples = length(sampleTimes);

for i = 1:3
%     posDiffAll(:,i) = smooth(posDiffAll(:,i), 5);
end

posDiffAll = posDiffAll*1000/sampleDtVision;

nnInput = [posDiffAll(:, 1:end-3) velLocalAll(:, 1:end-3)];
nnTarget = posDiffAll(:, end-2:end);

nnInput = nnInput';
nnOutput = nnTarget';

net = fitnet(20);
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
net.trainParam.max_fail = 20;
net.trainParam.min_grad = 1e-10;
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

% subplot(2,1,1);
% histogram(errors(1:2,:))
% title('XY Error Histogram');
% axis tight;
% 
% subplot(2,1,2);
% histogram(errors(3,:))
% title('W Error Histogram');
% axis tight;

netVelLocal = net(nnInput)';
visVelLocal = nnTarget;
outVelEncLocal = velLocalAll(:,end-2:end);
sT = sampleTimes;

% outVelEncLocal = out.vel.enc.local(:,1:3);
% netVelLocal = net(outVelEncLocal')'*1000/sampleDtVision;
% visVelLocal = in.vel.vis.local(:,1:3);
% sT = (0:length(visVelLocal)-1)*sampleDtVision/1000;

sp1 = subplot(3,1,1);
plot(sT, visVelLocal(:,1), 'g--', sT, netVelLocal(:,1), 'r', sT, outVelEncLocal(:,1), 'b:');

sp2 = subplot(3,1,2);
plot(sT, visVelLocal(:,2), 'g--', sT, netVelLocal(:,2), 'r', sT, outVelEncLocal(:,2), 'b:');

sp3 = subplot(3,1,3);
plot(sT, visVelLocal(:,3), 'g--', sT, netVelLocal(:,3), 'r', sT, outVelEncLocal(:,3), 'b:');

linkaxes([sp1, sp2, sp3], 'x');
