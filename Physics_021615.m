%   Start this first if using hub-server
%   start after if using local MC server

%   this variation puts CANbus payload on natural boundaries
clc
clear all

%   This script will evaluate the position, velocity, and acceleration of
%   the glider based on the tension and the state from the Master Controler

global u state fG ydotdot xdotdot xyint
global Vs V1 LoD Kd deltp lambda delta MgoMp

import java.net.*
import java.io.*
import java.lang.*
import GlassControlPanel.glasscontrolpanel.*

TICSPERSECOND = 16;
Tdsp = 50;
tinc = 1/TICSPERSECOND;

format compact
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  LAUNCH PARAMETERES  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%conversion factors
m2f = 3.281;    %   Converstion factor for meters to feet
m2k = 1.9438;   %   Conersion factor for m/s to knots
G = 9.81;       %   Accel due to gravity

%Parameters
Mg = 600;               %   Glider mass kg
Vs = 36 / m2k;          %   1G stall speed
V1 = 1.3333 * Vs;       %   V1 Speed
Xi = 999;               %   Run Length (m)
LoD = 10;               %   L/D
Fg = 1.0;               %   Ground run tension factor
Fc = 1.3;               %   Climb tension factor
Fp = 10;
Vtr = 40/m2k;           %   Velocity to start tapering tension
Vmx = 81/m2k;           %   Velocity where tension reaches 0
Ttu = 1;                %   Initial taper up period
%   Tension factor ramp up period
Kd = 0.08;              %   Derivative gain
lambda = G / V1^2;      %   Lambda lift factor
delta = lambda/LoD;     %   Delta drag factor
Mp = 100;               %   Parachtue and cable mass (and some drum)
MgoMp = Mg/Mp;          %   Ratio of glider to parachute mass
deltp = 0.1 / MgoMp;    %   Parachtue drag parameter

xdotinit = 0;
ydotinit = 0;
yinit = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DEFINE MESSAGE IDs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ID_OFFSET = 2^21;

TIME_MESSAGE_ID = 256 * ID_OFFSET;
STATE_MESSAGE_ID = 304 * ID_OFFSET;
TORQUE_MESSAGE_ID = 300 * ID_OFFSET;
MOTOR_MESSAGE_ID = 296 * ID_OFFSET;         % inconsistnet with Excel
CONTROL_LEVER_MESSAGE_RMT_ID = 328 * ID_OFFSET; 
CONTROL_LEVER_MESSAGE_LCL_ID = 329 * ID_OFFSET;
CP_INPUTS_RMT_ID = 294 * ID_OFFSET; 
CP_INPUT_LCL_ID = 296 * ID_OFFSET;
CP_OUTPUT_ID = 336 * ID_OFFSET;
TENSION_MESSAGE_ID = 448 * ID_OFFSET;       
CABLE_ANGLE_MESSAGE_ID = 464 * ID_OFFSET;   
LAUNCH_PARAM_MESSAGE_ID = 327 * ID_OFFSET;  % inconsistnet with Excel
PARAM_REQUEST_MESSAGE_ID = 312 * ID_OFFSET;

CABLE_ANGLE_MESSAGE_RATE = 8;
CABLE_ANGLE_MESSAGE_MOD = 2;


Tmax =100;
num_tim_intrv = Tmax/tinc + 1;  %   Maximum number of time steps

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   DEFINE MESSAGES, OFFSETS, SCALING, and INITIAL VALUES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%   prepare messages for before the launch begins

%tensionScale = 0.25;
%tensionOffset = 1024;
torqueScale = 1/32;
torqueToTension = 20;

tensionMessage = glasscontrolpanel.CanCnvt();
tensionMessage.id = TENSION_MESSAGE_ID;


motorToDrum = 7;
drumRadius = 0.4;
%motorSpeedScale = 1/128;

motorMessage = glasscontrolpanel.CanCnvt();
motorMessage.id = MOTOR_MESSAGE_ID;

%cableAngleScale = 0.5;
%cableAngleOffset = 40;  %   offset in lsbs

cableAngleMessage = glasscontrolpanel.CanCnvt();
cableAngleMessage.id = CABLE_ANGLE_MESSAGE_ID;

paramMessage = glasscontrolpanel.CanCnvt();
paramMessage.id = LAUNCH_PARAM_MESSAGE_ID;

canIn = glasscontrolpanel.CanCnvt();
msg = '';   %   Is this needed?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CONNECT TO SOCKET!%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic; display('Setting up sockets'); % 1000*toc, tic;

if 0
%      socket = java.net.Socket('192.168.1.', 32123);    % hub-server on
%     George home
%     socket = java.net.Socket('147.222.165.75', 32123);    % hub-server on Jacob
socket = java.net.Socket('192.168.1.7', 32123);
else
    socket = java.net.Socket('127.0.0.1', 32123); % client to local server
end

socket.setTcpNoDelay(true);
instream = BufferedReader(InputStreamReader(socket.getInputStream()));
outstream = OutputStreamWriter(socket.getOutputStream());

launchInProgressFlag = 0;
launchNumber = 0;

%   Multiple sequential simulation loop

while(true)
    %   variables that need to be reset before each run
    state = 1;
    i = 1;  %   index counter for data storage
    launchNumber = launchNumber + 1  % launch number counter
    rdot = 0;
    motorSpeed = 0;
    cableAngle = 0;
    torqueMsgFlag = 0;
    fracTime = 0;
    
    t1 = 0;
  
    fG = 0;
    ydotdot = 0;
    xdotdot = 0;
    xyint = [Xi xdotinit yinit ydotinit];
        
    %   initialize data for all CANbus sensor messages
    
    %tensionMessage.set_short(tensionOffset, 0); %   tension
    tensionMessage.set_halffloat(0.0, 0); %   tension
    tensionMessage.set_byte (0, 2);  %   status
    %tensionMessage.set_byte (fracTime, 3);  %   fracTime degug
    tensionMessage.dlc = 3 + 0;             %   is this needed
    
    %motorMessage.set_short(motorSpeed/motorSpeedScale,0);%  motor speed
    motorMessage.set_halffloat(motorSpeed, 0);
    motorMessage.set_halffloat(0.0, 2);    %   revolutions not implemented
    motorMessage.set_halffloat(0.0, 4);     %   temp not implemented
    motorMessage.set_byte(0, 6); %   status byte
    %motorMessage.set_byte(fracTime ,6); %   fracTime debug
    motorMessage.dlc = 7 + 0;           %   is this needed?
    
    %cableAngleMessage.set_byte(((cableAngle / cableAngleScale) ...
    %    + cableAngleOffset), 0); % cable angle
    cableAngleMessage.set_halffloat(cableAngle, 0);
    cableAngleMessage.set_byte(0, 2); % status
    %cableAngleMessage.set_byte(fracTime, 3);    %   fracTime debug
    cableAngleMessage.dlc = 3 + 0;              %   is this needed?
    
    %    Preallocate data variable for maximum number of rows
    data = zeros(num_tim_intrv, 8);
    data(i, :) = [0 0 0 Xi 0 0 0 0];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   Simulation Loop
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(state ~= 0)
%         display('Waiting for next message'); 1000*toc, tic;
        %   read next message
        msg = instream.readLine();
        %      display('message received');  1000*toc, tic;
        canIn.convert_msgtobin(msg); %  Parse the string into the proper CAN message
        
        %   Pick out the IDs that we are interested in
%         canIn.id/ID_OFFSET
        switch (canIn.id)
            case TIME_MESSAGE_ID
                if (canIn.dlc == 1)
                    fracTime = canIn.get_ubyte(0);
                else
                    fracTime = 0;
                end
                %display('Time message');  
                %   send send sensor messages from last simulation step
 %tensionMessage.set_byte (fracTime, 3);  %   fracTime degug
                tensionMessage.dlc = 3 + 0;             %   is this needed
                outstream.write(tensionMessage.msg_prep());
                
 %motorMessage.set_byte(fracTime ,6); %   fracTime debug
                motorMessage.dlc = 7 + 0;           %   is this needed?
                outstream.write(motorMessage.msg_prep());
                
                if mod(fracTime + CABLE_ANGLE_MESSAGE_MOD , ...
                        TICSPERSECOND/CABLE_ANGLE_MESSAGE_RATE) == 0
 %cableAngleMessage.set_byte(fracTime, 2);    %   fracTime debug
                    cableAngleMessage.dlc = 3 + 0;              %   is this needed?
                    outstream.write(cableAngleMessage.msg_prep());
               end
               outstream.flush();
                %   display('Sensor messages sent'); [1000*toc], tic;                
                
            case TORQUE_MESSAGE_ID
                receivedTorque = canIn.get_short(0)* torqueScale;
                %   display('Torque message'); [1000*toc j receivedTorque], tic;
                torqueMsgFlag = 1;                
                
            case PARAM_REQUEST_MESSAGE_ID
                % send parameter response message (simulating host controller)
                % if connected to real host remove the following two lines
                display('Parameter Request Message Received');
                paramMessage.msg_prep;
                outstream.write(paramMessage.msg_prep())
                outstream.flush();
                %   reset timers to plot from time 0
                i = 1;
                t1 = 0;
                launchInProgressFlag = 1;                
                                
            case STATE_MESSAGE_ID
                receivedState = canIn.get_byte(0)
%                 display([receivedState]);
                if (receivedState == 1)
                    %   dump unused data rows
                    data = data(1:i, :);
                    
                    %  end at recovery state and plot results
                    figure(1)
                    clf
                    pltrslts(data, i, Mg, Tdsp);
                    
                    subplot(2, 1, 1)
                    legend( 'Cable Speed (knots * 10)', ...
                        'Airspeed (knots * 10)', 'Tension (kgf)',...
                        'Height (m)', 'Cable Angle * 10 (deg)', ...
                        'location', 'east')
                    xlabel('Time (seconds)')
                    grid on
                    lmts = axis;
                    axis([0 lmts(2) 0 lmts(4)]);
                    zoom on
                    
                    subplot(2, 1, 2)
                    lmts = axis;
                    axis([0 lmts(2) 0 lmts(4)])
                    xlabel('x (m)')
                    ylabel('y (m)')
                    grid on
                    
                    text(lmts(2)/10, lmts(4)/7, ['Launch Number: ' ...
                        sprintf('%2d',launchNumber)]);
                    
                    pause(1)
                    
                    state = 0;  %    break to outer loop and reset for next launch
                    launchInProgressFlag = 0;
                end
                
            case CONTROL_LEVER_MESSAGE_LCL_ID
                
                
            otherwise 
                %   later this will mean unused ID and just ignored
                %disp('Unknown ID')
                %disp(canIn.id / ID_OFFSET);
                
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if ((torqueMsgFlag == 1) && (launchInProgressFlag == 1))
            %         display('Simulation step started'); [1000*toc], tic;            
            fG = (receivedTorque * torqueToTension) / Mg;
            
            %   Use the Runge Kutta method to advance y'',x'',x',y'
            %   for the next interval
            u = [xyint state];
            [tode, xy] = ode45(@Physics_Ode, [t1 t1 + tinc], xyint);
            %   compute the dotdot variables for the end of the current
            %   interval
            xyint = xy(end, :);
            u = [xyint state];
            dotdot = FMA_Parachute(fG, u, lambda, delta, Kd, MgoMp, deltp);
            %   Store the results for this last interval and prepare for the
            %   next interval
            i = i + 1;
            t1 = t1 + tinc;
            data(i, :) = [dotdot, fG, xyint, t1];
            %   determine state for the next simulation interval
            state = State_Parachute(data(i, :));
            %         display('Simulation step finished'); [1000*toc], tic;
            %  if (mod(round(t1 * TICSPERSECOND), 16) == 0);
                if (floor(t1 * 4) - floor((t1 - tinc) * 4) == 1)
                %   plot running results
                    pltrslts(data, i, Mg, Tdsp);
                %             display('Ploting complete'); [1000*toc], tic;
                end
            
            %   Values needed for messages to Master Controller
            x = data(i, 4);
            y = data(i, 6);
            xdot = data(i, 5);
            ydot = data(i, 7);
            rsq = x.^2 + y.^2;
            r = sqrt(rsq);
            rdot = -(x .* xdot + y .* ydot)./ r;
%             rdotdot = -() / r;
            
            %   Prepare to send the sesor messages on reception
            %   of next time message
            motorSpeed = motorToDrum * rdot / (2 * pi * drumRadius);
            if state == 8
                motorSpeed = 0;
            end            
            %motorMessage.set_short(motorSpeed/motorSpeedScale, 0);  % speed
            motorMessage.set_halffloat(motorSpeed, 0); %    motor speed
            motorMessage.set_halffloat(0.0, 2); % revolutions (not implemented)
            motorMessage.set_halffloat(0.0, 4); % temperature (not used)
            motorMessage.set_byte(0, 6); % status
            motorMessage.dlc = 7;   %   is this needed?
            
            %   Cable Angle
            theta = atan2(y, x);
            cableAngle = theta * 180/pi;    %   in degrees
            cableAngleMessage.set_halffloat(cableAngle, 0);
            %cableAngleMessage.set_byte((cableAngle / cableAngleScale) ...
            %    + cableAngleOffset, 0);
            cableAngleMessage.set_byte(0,2); %  Status
            cableAngleMessage.dlc = 3;  %   is this needed?
            
            %   Tension
            tensionMessage.set_halffloat(data(i, 3) * Mg, 0);
            %tensionMessage.set_short((data(i, 3) * Mg / tensionScale) ...
            %    + tensionOffset, 0);
            tensionMessage.set_byte(0, 2); %    Status
            tensionMessage.dlc = 3; %   is this needed
            
            torqueMsgFlag = 0;
            %         display('Sensor messages formating completed'); [1000*toc], tic;
        end
    end
     display('End of Launch')
end
outstream.close();
instream.close();
socket.close()
return