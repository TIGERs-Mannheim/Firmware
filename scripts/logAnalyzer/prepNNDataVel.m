clear;
clc;

visionDelay = 70; % [ms]
sampleDt = 5; % [ms]
numPastSamples = 5;
hiddenNeurons = 8;

logData = loadLogFile('trial12.dat');
[in, state, out, sampleTimes] = processCtrlData(logData);
[debug] = processDebugData(logData, sampleTimes);

outVelLocal = out.vel.enc.local(:,1:3);
inVelLocal = in.vel.vis.local(:,1:3);
inPosGlobal = in.pos.vis.global; 

% shift vision data to align it with "now" time
inPosGlobal = circshift(inPosGlobal, -visionDelay);
inVelLocal = circshift(inVelLocal, -visionDelay);

% remove circular shifted data at end => useless
inPosGlobal(end-visionDelay+1:end,:) = [];
inVelLocal(end-visionDelay+1:end,:) = [];
outVelLocal(end-visionDelay+1:end,:) = [];
sampleTimes(end-visionDelay+1:end,:) = [];

posGlobalVision = inPosGlobal(1:sampleDt:end, :);
velLocalVision = inVelLocal(1:sampleDt:end, :);
velLocalEnc = outVelLocal(1:sampleDt:end, :);
sampleTimes = sampleTimes(1:sampleDt:end, :);

velGlobalVision = (posGlobalVision - circshift(posGlobalVision, 1)) * 1000/sampleDt;
velGlobalVision(1,:) = [];
% velLocalEnc(1,:) = [];
% sampleTimes(1) = [];

numSamples = length(velLocalEnc);

% velLocalVision = velGlobalVision;
% 
% % rotate position to local frame
% for i = 1:numSamples
% 	angle = pi/2-inPosGlobal(i, 3);
% 	rotMat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
%     
%     velLocalVision(i, 1:2) = (rotMat*velGlobalVision(i, 1:2)')';
% end

% for i = 1:3
%     velLocalVision(:,i) = smooth(velLocalVision(:,i), 10);
% end

% plot(sampleTimes, velLocalEnc(:,1), sampleTimes, velLocalVision(:,1));

X = tonndata(velLocalEnc,false,false);
T = tonndata(velLocalVision,false,false);

% Choose a Training Function
% For a list of all training functions type: help nntrain
% 'trainlm' is usually fastest.
% 'trainbr' takes longer but may be better for challenging problems.
% 'trainscg' uses less memory. Suitable in low memory situations.
trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.

% Create a Nonlinear Autoregressive Network with External Input
inputDelays = 0:20;
feedbackDelays = 10:20;
hiddenLayerSize = hiddenNeurons;
net = narxnet(inputDelays,feedbackDelays,hiddenLayerSize,'open',trainFcn);

% Prepare the Data for Training and Simulation
% The function PREPARETS prepares timeseries data for a particular network,
% shifting time by the minimum amount to fill input states and layer
% states. Using PREPARETS allows you to keep your original time series data
% unchanged, while easily customizing it for networks with differing
% numbers of delays, with open loop or closed loop feedback modes.
[x,xi,ai,t] = preparets(net,X,{},T);

% Setup Division of Data for Training, Validation, Testing
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
net.trainParam.max_fail = 20;

% Train the Network
[net,tr] = train(net,x,t,xi,ai);

% Test the Network
outputs = net(x,xi,ai);
errors = gsubtract(t, outputs);
perf = perform(net, t, outputs)

errors = fromnndata(errors, true, false, false);

subplot(2,1,1);
histogram(errors(:,1:2))
title('XY Error Histogram');
axis tight;

subplot(2,1,2);
histogram(errors(:,3))
title('W Error Histogram');
axis tight;
