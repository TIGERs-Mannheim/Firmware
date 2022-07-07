function [ logData, in, out, ref, skill, sampleTimes, debug] = LogFileViewer()
%LOGFILEVIEWER Graphical viewer for log file data
%   Awesome tool for data analysis

clc;
clear;

global state;
global filename;
global pathname;

loadFile(0,0);

figure('Name', strcat('Logfile Viewer: ', filename), 'NumberTitle', 'Off');
m = uimenu('Label', 'Dataset');
uimenu(m, 'Label', 'Load File', 'Callback', @loadFile);
uimenu(m, 'Label', 'Global Position (X of Y)', 'Callback', @showGlobalPosition);
uimenu(m, 'Label', 'Global Position (XYW of t)', 'Callback', @showGlobalPositionXYW);
uimenu(m, 'Label', 'Local Position (Y of X)', 'Callback', @showLocalPositionXY);
uimenu(m, 'Label', 'Local Position (XYW of t)', 'Callback', @showLocalPositionXYW);
uimenu(m, 'Label', 'Vision Delta T', 'Callback', @showVisionDeltaT);
uimenu(m, 'Label', 'Local Velocity', 'Callback', @showLocalVelocity);
uimenu(m, 'Label', 'Local Velocity 2', 'Callback', @showLocalVelocity2);
uimenu(m, 'Label', 'Motor Velocity', 'Callback', @showMotorVelocities);
uimenu(m, 'Label', 'Motor Voltages', 'Callback', @showMotorVoltages);
uimenu(m, 'Label', 'Motor Currents', 'Callback', @showMotorCurrents);
uimenu(m, 'Label', 'Motor Torques', 'Callback', @showMotorTorques);
uimenu(m, 'Label', 'Acceleration', 'Callback', @showAcceleration);
uimenu(m, 'Label', 'IMU', 'Callback', @showIMU);
uimenu(m, 'Label', 'Accel. Velocity', 'Callback', @showAccelerometerVelocity);
uimenu(m, 'Label', 'Jerk', 'Callback', @showJerk);
uimenu(m, 'Label', 'Gyro FFT', 'Callback', @showGyrFFT);
uimenu(m, 'Label', 'Acc. FFT', 'Callback', @showAccFFT);
ctrlLocal = uimenu(m, 'Label', 'Ctrl: Local Vel');
uimenu(ctrlLocal, 'Label', 'XY', 'Callback', @showVelCtrlXY);
ctrlGlobal = uimenu(m, 'Label', 'Ctrl: Global Pos');
uimenu(ctrlGlobal, 'Label', 'XY', 'Callback', @showPosCtrlXY);
uimenu(ctrlGlobal, 'Label', 'W', 'Callback', @showPosCtrlW);
uimenu(m, 'Label', 'Command Response', 'Callback', @showCmdResponse);
uimenu(m, 'Label', 'Slip', 'Callback', @showSlip);
uimenu(m, 'Label', 'Motor Vel Error', 'Callback', @showMotorVelocityError);
uimenu(m, 'Label', 'Kicker & Dribbler', 'Callback', @showKickerDribbler);
uimenu(m, 'Label', 'Dribbler Details', 'Callback', @showDribbler);
uimenu(m, 'Label', 'Network Statistics', 'Callback', @showNetStats);
uimenu(m, 'Label', 'Power Consumption', 'Callback', @showPower);
uimenu(m, 'Label', 'IR Array', 'Callback', @showIRArray);
uimenu(m, 'Label', 'Robot Task Timings', 'Callback', @showTimings);

showLocalVelocity2(0, 0);

function loadFile(~, ~)
    [filename, pathname] = uigetfile('*.dat');
    logData = loadLogFile(fullfile(pathname, filename));
    [in, state, out, sampleTimes] = processCtrlData(logData);
    [ref] = processRefData(logData, sampleTimes);
    [skill] = processSkillData(logData, sampleTimes);
	[debug] = processDebugData(logData, sampleTimes);
end

function showLocalPositionXY(~, ~)
    subplot(1, 1, 1);
    plot(in.pos.enc.local(:,1), in.pos.enc.local(:,2), ...
        in.pos.vis.local(:,1), in.pos.vis.local(:,2));
    axis equal
    legend('Encoder', 'Vision');
    title('Local Position (Vision+Encoder)');
    xlabel('X [m]');
    ylabel('Y [m]');
end

function showVisionDeltaT(~, ~)
    subplot(1, 1, 1);
    plot(sampleTimes, in.pos.vis.dt);
    title('Vision Delta Time');
    axis tight
    xlabel('t [s]');
    ylabel('t [s]');
end

function showLocalPositionXYW(~, ~)
	sp1 = subplot(2, 1, 1);
	plot(sampleTimes, in.pos.enc.local(:,1), 'b-', sampleTimes, in.pos.vis.local(:,1), 'c:', ...
		sampleTimes, state.pos.local(:,1), 'k-',...
		sampleTimes, in.pos.enc.local(:,2), 'r-', sampleTimes, in.pos.vis.local(:,2), 'm:', ...
		sampleTimes, state.pos.local(:,2), 'g-');
	legend('X Enc', 'X Vision', 'X State', 'Y Enc', 'Y Vision', 'Y State');
	title('XY Position (Vision+Encoders)');
	xlabel('t [s]');
	ylabel('s [m]');
	axis tight

    sp2 = subplot(2, 1, 2);
    plot(sampleTimes, in.pos.enc.local(:,3), 'b-', sampleTimes, in.pos.vis.local(:,3), 'r:', ...
		sampleTimes, in.pos.gyr.local(:,3), 'g-', sampleTimes, state.pos.local(:,3), 'k-')
    axis tight
    legend('Encoder', 'Vision', 'Gyro', 'State');
    title('Local Orientation');
    xlabel('t [s]');
    ylabel('w [rad]');
    
    linkaxes([sp1, sp2], 'x');
end

function showGlobalPositionXYW(~, ~)
	sp1 = subplot(2, 1, 1);
	plot(sampleTimes, in.pos.vis.global(:,1), 'b:', ...
		sampleTimes, state.pos.global(:,1), 'k-',...
		sampleTimes, in.pos.vis.global(:,2), 'm:', ...
		sampleTimes, state.pos.global(:,2), 'g-');
	legend('X Vision', 'X State', 'Y Vision', 'Y State');
	title('XY Position (Vision+State)');
	xlabel('t [s]');
	ylabel('s [m]');
	axis tight

    sp2 = subplot(2, 1, 2);
    plot(sampleTimes, in.pos.enc.local(:,3), 'b-', sampleTimes, in.pos.vis.global(:,3), 'r:', ...
		sampleTimes, in.pos.gyr.local(:,3), 'g-', sampleTimes, state.pos.global(:,3), 'k-')
    axis tight
    legend('Encoder', 'Vision', 'Gyro', 'State');
    title('Global Orientation');
    xlabel('t [s]');
    ylabel('w [rad]');
    
    linkaxes([sp1, sp2], 'x');
end

function showMotorVelocities(~, ~)
	vmin = min(min(in.mot.vel));
	vmax = max(max(in.mot.vel));
	vlim = [vmin vmax];
    
    decim = 100;
	
    sp1 = subplot(2, 2, 2);
    plot(sampleTimes, in.mot.vel(:,1), 'b-', sampleTimes, smooth(circshift(in.mot.vis.vel(:,1),-50),decim), 'g--', ...
        sampleTimes, out.mot.vel(:,1), 'r-');
    legend('Encoder', 'Vision', 'Setpoint');
    title('M1 (Front Right)');
    ylabel('v [Motor rad/s]');
    xlabel('t [s]');
	axis tight
    grid minor
	ylim(vlim);    

    sp2 = subplot(2, 2, 1);
    plot(sampleTimes, in.mot.vel(:,2), 'b-', sampleTimes, smooth(circshift(in.mot.vis.vel(:,2),-50),decim), 'g--', ...
        sampleTimes, out.mot.vel(:,2), 'r-');
    title('M2 (Front Left)');
    ylabel('v [Motor rad/s]');
    xlabel('t [s]');
	axis tight
    grid minor
	ylim(vlim);

    sp3 = subplot(2, 2, 3);
    plot(sampleTimes, in.mot.vel(:,3), 'b-', sampleTimes, smooth(circshift(in.mot.vis.vel(:,3),-50),decim), 'g--', ...
        sampleTimes, out.mot.vel(:,3), 'r-');
    title('M3 (Rear Left)');
    ylabel('v [Motor rad/s]');
    xlabel('t [s]');
	axis tight
    grid minor
	ylim(vlim);

    sp4 = subplot(2, 2, 4);
    plot(sampleTimes, in.mot.vel(:,4), 'b-', sampleTimes, smooth(circshift(in.mot.vis.vel(:,4),-50),decim), 'g--', ...
        sampleTimes, out.mot.vel(:,4), 'r-');
    title('M4 (Rear Right)');
    ylabel('v [Motor rad/s]');
    xlabel('t [s]');
	axis tight
    grid minor
	ylim(vlim);
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showMotorVoltages(~, ~)
    bat = max(in.bat.vol);
    ulim = [-bat-1 bat+1];
    
    ke = 0.0251;
    if isfield(logData, 'ctrl_drive_train') && size(logData.ctrl_drive_train.data, 1) > 0
        ke = logData.ctrl_drive_train.data(1,6);
        disp('Using Ke from logfile');
    else
    end
    
    sp1 = subplot(2, 2, 2);
    plot(sampleTimes, in.mot.vel(:,1)*ke, sampleTimes, out.mot.vol(:,1),...
        sampleTimes, in.bat.vol, sampleTimes, -in.bat.vol);
    legend('Back-EMF', 'Output');
    title('M1 (Front Right)');
    ylabel('U [V]');
    xlabel('t [s]');
    grid on;
    axis tight;
	ylim(ulim);    

    sp2 = subplot(2, 2, 1);
    plot(sampleTimes, in.mot.vel(:,2)*ke, sampleTimes, out.mot.vol(:,2),...
        sampleTimes, in.bat.vol, sampleTimes, -in.bat.vol);
    title('M2 (Front Left)');
    ylabel('U [V]');
    xlabel('t [s]');
    grid on;
	axis tight
	ylim(ulim);

    sp3 = subplot(2, 2, 3);
    plot(sampleTimes, in.mot.vel(:,3)*ke, sampleTimes, out.mot.vol(:,3),...
        sampleTimes, in.bat.vol, sampleTimes, -in.bat.vol);
    title('M3 (Rear Left)');
    ylabel('U [V]');
    xlabel('t [s]');
    grid on;
	axis tight
	ylim(ulim);

    sp4 = subplot(2, 2, 4);
    plot(sampleTimes, in.mot.vel(:,4)*ke, sampleTimes, out.mot.vol(:,4),...
        sampleTimes, in.bat.vol, sampleTimes, -in.bat.vol);
    title('M4 (Rear Right)');
    ylabel('U [V]');
    xlabel('t [s]');
    grid on;
	axis tight
	ylim(ulim);
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showMotorTorques(~, ~)
    ke = 0.0251;
    km = ke;
    R = 0.5;
    if isfield(logData, 'ctrl_drive_train') && size(logData.ctrl_drive_train.data, 1) > 0
        ke = logData.ctrl_drive_train.data(1,6);
        km = logData.ctrl_drive_train.data(1,4);
        R = logData.ctrl_drive_train.data(1,8);
        disp('Using Ke from logfile');
    else
    end
    
    torque = (out.mot.vol(:,1:4) - in.mot.vel(:,1:4)*ke) .* km/R;
    
    smoothVal = 50;
    
    sp1 = subplot(2, 2, 2);
    plot(sampleTimes, smooth(torque(:,1),smoothVal));
    legend('Torque');
    title('M1 (Front Right)');
    ylabel('M [Nm]');
    xlabel('t [s]');
    grid on;
    axis tight;

    sp2 = subplot(2, 2, 1);
    plot(sampleTimes, smooth(torque(:,2),smoothVal));
    title('M2 (Front Left)');
    ylabel('M [Nm]');
    xlabel('t [s]');
    grid on;
	axis tight

    sp3 = subplot(2, 2, 3);
    plot(sampleTimes, smooth(torque(:,3),smoothVal));
    title('M3 (Rear Left)');
    ylabel('M [Nm]');
    xlabel('t [s]');
    grid on;
	axis tight

    sp4 = subplot(2, 2, 4);
    plot(sampleTimes, smooth(torque(:,4),smoothVal));
    title('M4 (Rear Right)');
    ylabel('M [Nm]');
    xlabel('t [s]');
    grid on;
	axis tight
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showMotorCurrents(~, ~)
    subplot(1, 1, 1);
    plot(sampleTimes, in.mot.cur(:,1), 'r-',...
        sampleTimes, in.mot.cur(:,2), 'g-', ...
        sampleTimes, in.mot.cur(:,3), 'b-', ...
        sampleTimes, in.mot.cur(:,4), 'k-', ...
        sampleTimes, out.mot.cur(:,1), 'r--',...
        sampleTimes, out.mot.cur(:,2), 'g--', ...
        sampleTimes, out.mot.cur(:,3), 'b--', ...
        sampleTimes, out.mot.cur(:,4), 'k--');
    legend('M1 meas', 'M2 meas', 'M3 meas', 'M4 meas', ...
        'M1 set', 'M2 set', 'M3 set', 'M4 set');
    title('Motor currents');
    ylabel('I [A]');
    xlabel('t [s]');
    grid on;
    axis tight;
end

function showGyrFFT(~, ~)
    firstCol = find(strcmp(logData.sensors.names, 'gyr_updated'));
    g = logData.sensors.data(:,firstCol+1:firstCol+4);

    subplot(2, 1, 1);
    plot(sampleTimes, in.vel.gyr.local(:,3));
    title('Gyro Raw Data');
    xlabel('t [s]');
    ylabel('Raw');
    grid on;
    grid minor;
	axis tight;

    Fs = 1000;
    L = length(g);

    NFFT = 2^nextpow2(L);
    Yx = fft(g(:,4), NFFT)/L;
    f = Fs/2*linspace(0,1,NFFT/2+1);

    absYx = 2*abs(Yx(1:NFFT/2+1));

    subplot(2, 1, 2);
    plot(f(10:end), absYx(10:end));
    title('FFT Analysis');
    xlabel('F [Hz]');
    ylabel('Amplitude');
    grid on;
    grid minor;
	axis tight;
end

function showAccFFT(~, ~)
    firstCol = find(strcmp(logData.sensors.names, 'acc_updated'));
    g = logData.sensors.data(:,firstCol+1:firstCol+4);
    
    subplot(2, 1, 1);
    plot(sampleTimes, in.acc.acc.local(:,3));
    title('Accelerometer');
    xlabel('t [s]');
    ylabel('m/s^2');
    grid on;
    grid minor;
	axis tight;

    Fs = 1000;
    L = length(g);

    NFFT = 2^nextpow2(L);
    Yx = fft(g(:,4), NFFT)/L;
    f = Fs/2*linspace(0,1,NFFT/2+1);

    absYx = 2*abs(Yx(1:NFFT/2+1));

    subplot(2, 1, 2);
    plot(f(10:end), absYx(10:end));
    title('FFT Analysis');
    xlabel('F [Hz]');
    ylabel('Amplitude');
    grid on;
    grid minor;
	axis tight;
end

function showLocalVelocity(~, ~)
	sp1 = subplot(2, 1, 1);
	plot(sampleTimes, in.vel.enc.local(:,1), 'b-', sampleTimes, out.vel.enc.local(:,1), 'c:', ...
		sampleTimes, in.vel.enc.local(:,2), 'r-', sampleTimes, out.vel.enc.local(:,2), 'm:', ...
        sampleTimes, in.vel.vis.local(:,1), 'g--', sampleTimes, in.vel.vis.local(:,2), 'k--');
	legend('X Enc', 'X Setpoint', 'Y Enc', 'Y Setpoint', 'X Vision', 'Y Vision');
	title('XY Velocities (Encoders+Vision)');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight

	sp2 = subplot(2, 1, 2);
	plot(sampleTimes, in.vel.enc.local(:,3), 'g-', ...
		sampleTimes, in.vel.gyr.local(:,3), 'm-', ...
		sampleTimes, out.vel.enc.local(:,3), 'k:', ...
        sampleTimes, in.vel.vis.local(:,3), 'c--');
	legend('W Enc', 'W Gyro', 'W Setpoint', 'W Vision');
	title('Rotational Velocity (Encoders+Gyro+Vision)');
	xlabel('t [s]');
	ylabel('v [rad/s]');
	axis tight
    
    linkaxes([sp1, sp2], 'x');
end

function showLocalVelocity2(~, ~)
    sp1 = subplot(2, 2, 1);
	plot(sampleTimes, in.vel.enc.local(:,1), 'k:', ...
        sampleTimes, in.vel.vis.local(:,1), 'c-', ...
        sampleTimes, ref.vel.local(:,1), 'r:', ...
        sampleTimes, state.vel.local(:,1), 'b-');
	legend('Enc Raw', 'Vision', 'Traj', 'State');
	title('X Velocities (Encoders+Accelerometer+Vision)');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight
    grid on
    grid minor
	ylim([-7 7]);
    
	sp2 = subplot(2, 2, 2);
	plot(sampleTimes, in.vel.enc.local(:,2), 'k:', ...
        sampleTimes, in.vel.vis.local(:,2), 'c-', ...
        sampleTimes, ref.vel.local(:,2), 'r:', ...
        sampleTimes, state.vel.local(:,2), 'b-');
    legend('Enc Raw', 'Vision', 'Traj', 'State');
	title('Y Velocities (Encoders+Accelerometer+Vision)');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight
    grid on
    grid minor
	ylim([-7 7]);

	sp3 = subplot(2, 2, 3);
	plot(sampleTimes, in.vel.enc.local(:,3), 'g--', ...
		sampleTimes, in.vel.gyr.local(:,3), 'm--', ...
        sampleTimes, in.vel.vis.local(:,3), 'c-', ...
        sampleTimes, state.vel.local(:,3), 'b-');
	legend('Enc', 'Gyro', 'Vision', 'State');
	title('Rotational Velocity (Encoders+Gyro+Vision)');
	xlabel('t [s]');
	ylabel('v [rad/s]');
	axis tight
    grid on
    grid minor
	ylim([-30 30]);
    
	sp4 = subplot(2, 2, 4);
	plot(sampleTimes, in.vel.enc.local(:,4), 'g--', ...
        sampleTimes, in.vel.vis.local(:,4), 'c-', ...
        sampleTimes, state.vel.local(:,4), 'b-');
	legend('Enc', 'Vision', 'State');
	title('Norm(XY) Velocity (Encoders+Gyro+Vision)');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight
    grid on
    grid minor
	ylim([0 5]);
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showGlobalPosition(~, ~)
    subplot(1, 1, 1);
    plot(in.pos.vis.global(:,1), in.pos.vis.global(:,2), ...
		state.pos.global(:,1), state.pos.global(:,2),...
		in.pos.vis.global(1,1), in.pos.vis.global(1,2), 'x',...
		state.pos.global(1,1), state.pos.global(1,2), 'x');
    hold on;
    qiv = 1:50:length(sampleTimes);
    quiver(in.pos.vis.global(qiv,1), in.pos.vis.global(qiv,2), ...
        cos(in.pos.vis.global(qiv,3)), sin(in.pos.vis.global(qiv,3)), 'g--');
    hold off;
	axis equal
    legend('Vision', 'State');
    title('Global Position (Vision+State)');
    xlabel('X [m]');
    ylabel('Y [m]');
    
end

function showAcceleration(~, ~)
    sm = 1;
    
    subplot(1, 1, 1);
    plot(sampleTimes, smooth(in.acc.acc.local(:,1),sm), 'b-', sampleTimes, smooth(in.acc.acc.local(:,2),sm), 'r-', ...
        sampleTimes, smooth(in.acc.acc.local(:,3),sm), 'g-', ...
		sampleTimes, smooth(in.acc.vis.local(:,1),sm), 'c:', sampleTimes, smooth(in.acc.vis.local(:,2),sm), 'm:', ...
        sampleTimes, ref.acc.local(:,1), 'b--', sampleTimes, ref.acc.local(:,2), 'r--', ...
        sampleTimes, smooth(state.acc.local(:,1),sm), 'k--', sampleTimes, smooth(state.acc.local(:,2),sm), 'g--');
    axis tight;
    grid on;
    grid minor;
    legend('X Acc', 'Y Acc', 'Z Acc', 'X Vision', 'Y Vision', 'X Traj', 'Y Traj', 'X State', 'Y State');
    title('Acceleration');
    xlabel('t [s]');
    ylabel('a [m/s^2]');
end

function showIMU(~, ~)
    sp1 = subplot(2, 1, 1);
    plot(sampleTimes, in.vel.gyr.local(:,1), 'b-', sampleTimes, in.vel.gyr.local(:,2), 'r-', ...
        sampleTimes, in.vel.gyr.local(:,3), 'g-', ...
		sampleTimes, in.vel.vis.local(:,3), 'c:', ...
        sampleTimes, ref.vel.local(:,3), 'b--', ...
        sampleTimes, state.vel.local(:,3), 'k--');
    axis tight;
    grid on;
    grid minor;
    legend('X Gyr', 'Y Gyr', 'Z Gyr', 'Z Vision', 'Z Traj', 'Z State');
    title('Gyro');
    xlabel('t [s]');
    ylabel('v [rad/s]');
    
    sp2 = subplot(2, 1, 2);
    plot(sampleTimes, in.acc.acc.local(:,1), 'b-', sampleTimes, in.acc.acc.local(:,2), 'r-', ...
        sampleTimes, in.acc.acc.local(:,3), 'g-', ...
		sampleTimes, in.acc.vis.local(:,1), 'c:', sampleTimes, in.acc.vis.local(:,2), 'm:', ...
        sampleTimes, ref.acc.local(:,1), 'b--', sampleTimes, ref.acc.local(:,2), 'r--', ...
        sampleTimes, state.acc.local(:,1), 'k--', sampleTimes, state.acc.local(:,2), 'g--');
    axis tight;
    grid on;
    grid minor;
    legend('X Acc', 'Y Acc', 'Z Acc', 'X Vision', 'Y Vision', 'X Traj', 'Y Traj', 'X State', 'Y State');
    title('Accelerometer');
    xlabel('t [s]');
    ylabel('a [m/s^2]');
    
    linkaxes([sp1, sp2], 'x');
end

function showJerk(~, ~)
    subplot(1, 1, 1);
    plot(sampleTimes, smooth(in.jerk.acc.local(:,1), 100), 'b-', sampleTimes, smooth(in.jerk.acc.local(:,2), 100), 'r-', ...
        sampleTimes, ref.acc.local(:,1), 'b--', sampleTimes, ref.acc.local(:,2), 'r--');
    axis tight
    legend('X Acc', 'Y Acc', 'X Traj', 'Y Traj');
    title('Jerk');
    xlabel('t [s]');
    ylabel('j [m/s^3]');
end

function showAccelerometerVelocity(~, ~)
    subplot(1, 1, 1);
    plot(sampleTimes, in.vel.acc.local(:,1), 'b-', ...
        sampleTimes, in.vel.acc.local(:,2), 'r-', ...
        sampleTimes, state.vel.localAcc(:,1), 'c-', ...
        sampleTimes, state.vel.localAcc(:,2), 'm-');
    axis tight
    grid on
    grid minor
    legend('X Acc', 'Y Acc', 'X State', 'Y State');
    title('Integrated Accelerometer Velocity');
    xlabel('t [s]');
    ylabel('v [m/s]');
end

function showVelCtrlXY(~, ~)
	sp1 = subplot(2, 2, 1);
	plot(sampleTimes, state.vel.local(:,1), 'g-', sampleTimes, state.vel.local(:,2), 'k-', ...
        sampleTimes, in.vel.vis.local(:,1), 'g:', sampleTimes, in.vel.vis.local(:,2), 'k:', ...
		sampleTimes, ref.vel.local(:,1), 'c-', sampleTimes, ref.vel.local(:,2), 'm-', ...
		sampleTimes, skill.drive.vel.local(:,1), 'b-', sampleTimes, skill.drive.vel.local(:,2), 'r-');
	legend('X State', 'Y State', 'X Vision', 'Y Vision', 'X Ref', 'Y Ref', 'X Set', 'Y Set');
	title('XY Velocities (State+Vision+VelInt+Setpoint)');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight

	sp3 = subplot(2, 2, 3);
	plot(sampleTimes, state.vel.local(:,3), 'k:', ...
        sampleTimes, in.vel.vis.local(:,3), 'c--', ...
		sampleTimes, ref.vel.local(:,3), 'g-', ...
		sampleTimes, skill.drive.vel.local(:,3), 'r-');
	legend('W State', 'W Vision', 'W Ref', 'W Setpoint');
	title('Rotational Velocity (State+Vision+VelInt+Setpoint)');
	xlabel('t [s]');
	ylabel('v [rad/s]');
	axis tight

	sp2 = subplot(2, 2, 2);
	plot(sampleTimes, ref.acc.local(:,1), 'k:', ...
		sampleTimes, state.vel.local(:,1), 'g-', ...
		sampleTimes, skill.drive.vel.local(:,1), 'r-');
	legend('X Acc', 'X Vel State', 'X Vel Ref');
	title('X Control');
	xlabel('t [s]');
	ylabel('v [rad/s]');
	axis tight

	sp4 = subplot(2, 2, 4);
	plot(sampleTimes, skill.drive.modeXY(:,1), 'k-');
	title('Ctrl Mode');
	xlabel('t [s]');
	ylabel('Mode');
	axis tight
	ylim([-1 6]);
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showPosCtrlXY(~, ~)
	sp1 = subplot(2, 2, 1);
	plot(sampleTimes, state.pos.global(:,1), 'b-', ...
        sampleTimes, in.pos.vis.global(:,1), 'r:', ...
		sampleTimes, skill.drive.pos.global(:,1), 'g--', ...
        sampleTimes, ref.pos.global(:,1), 'm--');
	legend('State', 'Vision', 'Set', 'Traj');
	title('X Position');
	xlabel('t [s]');
	ylabel('p [m]');
	axis tight
    grid minor

	sp2 = subplot(2, 2, 2);
	plot(sampleTimes, state.pos.global(:,2), 'b-', ...
        sampleTimes, in.pos.vis.global(:,2), 'r:', ...
		sampleTimes, skill.drive.pos.global(:,2), 'g--', ...
        sampleTimes, ref.pos.global(:,2), 'm--');
	legend('State', 'Vision', 'Set', 'Traj');
	title('Y Position');
	xlabel('t [s]');
	ylabel('p [m]');
	axis tight
	grid minor

	sp3 = subplot(2, 2, 3);
	plot(sampleTimes, state.vel.global(:,1), 'b-', ...
        sampleTimes, ref.vel.global(:,1), 'm--');
    legend('State', 'Traj');
	title('X Velocity');
	xlabel('t [s]');
	axis tight
    grid minor

	sp4 = subplot(2, 2, 4);
    plot(sampleTimes, state.vel.global(:,2), 'b-', ...
        sampleTimes, ref.vel.global(:,2), 'm--');
    legend('State', 'Traj');
	title('Y Velocity');
	xlabel('t [s]');
	axis tight
    grid minor
	
	linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showPosCtrlW(~, ~)
	sp1 = subplot(2, 2, 1);
	plot(sampleTimes, state.pos.global(:,3), 'b-', ...
        sampleTimes, in.pos.vis.global(:,3), 'r:', ...
		sampleTimes, skill.drive.pos.global(:,3), 'g--', ...
        sampleTimes, ref.pos.global(:,3), 'm--');
	legend('W State', 'W Vision', 'W Set', 'W Traj');
	title('W Orientation (State+Vision+Setpoint)');
	xlabel('t [s]');
	ylabel('p [rad]');
	axis tight
    grid minor

	sp2 = subplot(2, 2, 3);
    plot(sampleTimes, state.vel.global(:,3), 'b-', ...
        sampleTimes, in.vel.gyr.local(:,3), 'k:', ...
        sampleTimes, ref.vel.global(:,3), 'm--');
    legend('State', 'Gyro', 'Traj');
	title('W Velocity');
	xlabel('t [s]');
	axis tight
    grid minor

	sp3 = subplot(2, 2, 2);
	plot([sampleTimes(1) sampleTimes(end)], [0 1], 'k-', ...
        [sampleTimes(1) sampleTimes(end)], [1 0], 'k-');
	title('-');
    axis tight

	sp4 = subplot(2, 2, 4);
	plot(sampleTimes, skill.drive.modeW(:,1), 'k-');
	title('Ctrl Mode');
	xlabel('t [s]');
	ylabel('Mode');
	axis tight
	ylim([-1 4]);
	
	linkaxes([sp1, sp3, sp4, sp2], 'x');
end

function showCmdResponse(~, ~)
    subplot(1, 1, 1);
    plot(sampleTimes, out.mot.vol(:,1), 'b-', sampleTimes, in.pos.vis.global(:,2), 'x', ...
		sampleTimes, in.pos.vis.global(:,1), 'o');
    axis tight
    legend('M1 Voltage', 'X Vision', 'Y Vision');
    title('Command response timing');
    xlabel('t [s]');
    ylabel('a [m/s^2]');
	grid on;
	grid minor;
end

function showSlip(~, ~)
	sp1 = subplot(2, 1, 1);
	plot(sampleTimes, in.vel.enc.local(:,4), 'b-');
% 	legend('X Enc', 'Y Enc');
	title('XY Norm Velocity (Encoders)');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight

	sp2 = subplot(2, 1, 2);
	plot(sampleTimes, in.slip.enc(:,1), 'g-');
% 	legend('Slip');
	title('Slip');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight
    
    linkaxes([sp1, sp2], 'x');
end

function showMotorVelocityError(~, ~)
    driveAngle = atan2(in.vel.vis.local(:,2), in.vel.vis.local(:,1))*180/pi;
    
    sp1 = subplot(2, 1, 1);
    plot(sampleTimes, driveAngle);
    legend('Vision');
    title('Local Driving Angle');
    ylabel('v [Motor RPS]');
    xlabel('t [s]');
	axis tight
    grid minor
	ylim([-190 190]);    

    sp2 = subplot(2, 1, 2);
    plot(sampleTimes, smooth(in.mot.vis.vel(:,1)-out.mot.vel(:,1),50), 'b-', ...
        sampleTimes, smooth(in.mot.vis.vel(:,2)-out.mot.vel(:,2),50), 'r-', ...
        sampleTimes, smooth(in.mot.vis.vel(:,3)-out.mot.vel(:,3),50), 'g-', ...
        sampleTimes, smooth(in.mot.vis.vel(:,4)-out.mot.vel(:,4),50), 'k-');
    legend('M1', 'M2', 'M3', 'M4');
    title('Motor Velocity Error');
    ylabel('v [Motor RPS]');
    xlabel('t [s]');
	axis tight
    grid minor
    linkaxes([sp1, sp2], 'x');
end

function showKickerDribbler(~, ~)
    sp1 = subplot(2, 2, 1);
	plot(sampleTimes, in.kicker.vol, 'r-');
	title('Capacitor Voltage');
	xlabel('t [s]');
	ylabel('U [V]');
	axis tight
    grid on
    grid minor
	ylim([-5 230]);
    
	sp2 = subplot(2, 2, 2);
	plot(sampleTimes, in.barrier.on, 'g-', ...
        sampleTimes, in.barrier.off, 'r-', ...
        sampleTimes, in.barrier.irq, 'k--');
    legend('On', 'Off', 'Interrupted');
	title('Barrier');
	xlabel('t [s]');
	ylabel('U [V]');
	axis tight
    grid on
    grid minor
	ylim([-0.1 2.0]);

	sp3 = subplot(2, 2, 3);
	plot(sampleTimes, skill.kicker.mode, 'g--', ...
		sampleTimes, skill.kicker.device, 'b--', ...
        sampleTimes, skill.kicker.speed, 'c-');
	legend('Mode', 'Device', 'Speed');
	title('Kicker Command');
	xlabel('t [s]');
	ylabel('v [m/s]');
	axis tight
    grid on
    grid minor
	ylim([-1 10]);
    
	sp4 = subplot(2, 2, 4);
	plot(sampleTimes, skill.dribbler.speed, 'g--', ...
        sampleTimes, in.dribbler.speed, 'b-');
	legend('Ref', 'Real');
	title('Dribbler');
	xlabel('t [s]');
	ylabel('v [RPM]');
	axis tight
    grid on
    grid minor
	ylim([-5000 20000]);
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showDribbler(~, ~)
    sp1 = subplot(2, 2, 1);
	plot(sampleTimes, skill.dribbler.speed, 'g--', ...
        sampleTimes, in.dribbler.speed, 'b-', ...
        sampleTimes, in.dribbler.auxSpeed, 'r-', ...
        sampleTimes, state.drib.vel, 'k-');
	legend('Ref', 'Hall', 'Model', 'State');
	title('Dribbler Speed');
	xlabel('t [s]');
	ylabel('v [rad/s]');
	axis tight
    grid on
    grid minor
	ylim([-500 2500]);
    
	sp2 = subplot(2, 2, 2);
	plot(sampleTimes, in.dribbler.temp, 'b-');
	title('Dribbler Temperature');
	xlabel('t [s]');
	ylabel('T [°C]');
	axis tight
    grid on
    grid minor
	ylim([0 100.0]);
    
    sp3 = subplot(2, 2, 3);
	plot(sampleTimes, in.dribbler.current, 'b-', ...
        sampleTimes, state.drib.cur, 'k-', ...
        sampleTimes, skill.dribbler.maxCurrent, 'r-');
	legend('Real', 'State', 'Limit');
	title('Dribbler Current');
	xlabel('t [s]');
	ylabel('I [A]');
	axis tight
    grid on
    grid minor
    
	sp4 = subplot(2, 2, 4);
	plot(sampleTimes, in.dribbler.voltage, 'b-');
	legend('Output');
	title('Dribbler Voltage');
	xlabel('t [s]');
	ylabel('U [V]');
	axis tight
    grid on
    grid minor
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function tCor = rolloverComp16(tIn)
    cor = cumsum([0; diff(tIn) < -2^15])*2^16;
    tCor = tIn + cor;
end

function showNetStats(~, ~)
    net = logData.net_stats.data;
    
    sp1 = subplot(2, 2, 1);
	plot(net(:,1), net(:,3), 'r-');
	title('RSSI');
	xlabel('t [s]');
	ylabel('P [dBm]');
	axis tight
    grid on
    grid minor
	ylim([-100 2]);
    
    for i = 4:10
        net(:,i) = rolloverComp16(net(:,i));
    end
    
    txPackets = conv([0; diff(net(:,4))], ones(1, 10), 'same');
    rxPackets = conv([0; diff(net(:,6))], ones(1, 10), 'same');
    rxPacketsLost = conv([0; diff(net(:,8))], ones(1, 10), 'same');
    
	sp2 = subplot(2, 2, 2);
	plot(net(:,1), txPackets, 'b-', ...
        net(:,1), rxPackets, 'r-', ...
        net(:,1), rxPacketsLost, 'k-');
    legend('TX', 'RX Good', 'RX Loss');
	title('RF Packets');
	xlabel('t [s]');
	ylabel('Number of Packets per Second');
	axis tight
    grid on
    grid minor
    
    txBytes = conv([0; diff(net(:,5))], ones(1, 10), 'same');
    rxBytes = conv([0; diff(net(:,7))], ones(1, 10), 'same');

	sp3 = subplot(2, 2, 3);
	plot(net(:,1), txBytes, 'b-', ...
        net(:,1), rxBytes, 'r-');
	legend('TX', 'RX');
	title('RF Bytes');
	xlabel('t [s]');
	ylabel('Bytes per Second');
	axis tight
    grid on
    grid minor

    txHLLost = conv([0; diff(net(:,9))], ones(1, 10), 'same');
    rxHLLost = conv([0; diff(net(:,10))], ones(1, 10), 'same');

	sp4 = subplot(2, 2, 4);
	plot(net(:,1), txHLLost, 'b-', ...
        net(:,1), rxHLLost, 'r-');
	legend('TX', 'RX');
	title('RF HL Packets Lost');
	xlabel('t [s]');
	ylabel('Number of lost packets per second');
	axis tight
    grid on
    grid minor
    
    linkaxes([sp1, sp2, sp3, sp4], 'x');
end

function showIRArray(~, ~)
    subplot(1, 1, 1);
    
    xCol = strcmp(logData.sensors.names, 'ir_ball_est_x');
    yCol = strcmp(logData.sensors.names, 'ir_ball_est_y');
    detectCol = strcmp(logData.sensors.names, 'ir_ballDetected');

    detect = logical(logData.sensors.data(:,detectCol));

    x = logData.sensors.data(:,xCol);
    y = logData.sensors.data(:,yCol);

    
    plot(x(detect), y(detect));
    xlim([-35 35]);
    ylim([15 50]);
    title('Ball Position Estimation');
    xlabel('X-Coordinate [mm]');
    ylabel('Y-Coordinate [mm]');
end

function showPower(~, ~)
    sp1 = subplot(2, 1, 1);
    
  	plot(sampleTimes, in.bat.vol, 'b-');
	title('Battery Voltage');
	xlabel('t [s]');
	ylabel('U [V]');
	axis tight
    grid on
    grid minor  
    
    sp2 = subplot(2, 1, 2);
  	plot(sampleTimes, in.bat.cur, 'r-');
	title('Current Consumption');
	xlabel('t [s]');
	ylabel('I [A]');
	axis tight
    grid on
    grid minor  
    
    linkaxes([sp1, sp2], 'x');
end

function showTimings(~, ~)
    sp1 = subplot(2, 1, 1);
    
    tTotal = debug.time.input + debug.time.est + debug.time.skill + ...
        debug.time.ctrl + debug.time.out + debug.time.misc;
    
  	plot(sampleTimes, debug.time.input, 'c--', ...
         sampleTimes, debug.time.est, 'b-', ...
         sampleTimes, debug.time.skill, 'g--', ...
         sampleTimes, debug.time.ctrl, 'r-', ...
         sampleTimes, debug.time.out, 'k--', ...
         sampleTimes, debug.time.misc, 'm--');
	title('Robot Task Timings');
    legend('Input', 'Estimation', 'Skill', 'Control', 'Output', 'Misc');
	xlabel('t [s]');
	ylabel('t [us]');
	axis tight
    grid on
    grid minor  
    ylim([0 1000]);

    sp2 = subplot(2, 1, 2);
    
  	plot(sampleTimes, tTotal, 'k-');
	title('Total Time');
	xlabel('t [s]');
	ylabel('t [us]');
	axis tight
    grid on
    grid minor  
    ylim([0 1000]);
    
    linkaxes([sp1, sp2], 'x');
end

end
