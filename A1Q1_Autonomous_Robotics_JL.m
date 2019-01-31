%% Assignment 1 Question 1 %%
clear all;
close all;

%% Set up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear mylego
mylego = legoev3('usb');          
leftMotor = motor(mylego, 'A');      %LEFT   % Set up motor
rightMotor = motor(mylego, 'D');     %RIGHT
resetRotation(leftMotor);                    % Reset motor rotation counter
resetRotation(rightMotor);

rightMotor.Speed = 0;
leftMotor.Speed = 0;
start(leftMotor);                             % Start motor
start(rightMotor);

mycolorsensor = colorSensor(mylego);        % Setup colour sensor
%intensity = readLightIntensity(mycolorsensor,'reflected'); --- Copy and
%paste this to read intensity from color sensor 

%% Global variables: State and Vehicle Description

radius = 0.0558; % wheel radius in meters
circumference = 2*3.14159*radius; %wheel circumference
L = 0.118/2.0; %Distance between wheels (Differential drive) in meters

X = 0.0;                                       %robot location X
Y = 0.0;                                       %robot location Y
headingAngle = 0.0;                            %robot Heading
XHistory = zeros(1000);
YHistory = zeros(1000);

tapeHeading = 0.0;


lastR1 = 0;
lastR2 = 0;

goalX = 0.0;                                   % goal location X
goalY = 0.0;                                   % goal location Y
goalAngle = 0.0;                               %goal heading angle

%PID Driving Paramaters
distTol = 0.1;                               %meters
vMax = 90;                                   % Motor speed
vMin = 20;
vStandard = 50;
kpD = 0.01;                               % P controller parameter: DRIVING
kdD = 0.02;                               % D controller parameter: DRIVING
enOld = 0;
en = 0;


%PID Turning Paramaters
angle_tolerance = 0.2;                              % radians
wMax = 30;                                   % Motor speed
wMin = 10;
wStandard = 15;
kpT = 0.1;                                % P controller parameter: TURNING
kdT = 0.02;                               % D controller parameter: TURNING
eOld = 0;
e = 0;

%-------------------------------------------

count = 0;

%% Operations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i = 0;
state = 1; % begin in state 1
ts = 0.1;% Sampling period (play with this)
scatter(0, 0);
hold on;
while readButton(ev3, 'center') == 0
    intensity = readLightIntensity(mycolorsensor,'reflected');
    r1 = readRotation(leftMotor);        % Read rotation counter in degrees
    r2 = readRotation(rightMotor);
    
    if state == 1 % Follow Pipe (Black tape) (proportional controller)
        desired = 35; %check this
        error = desired - intensity;
        xn = error * 0.7; 
        if error >= 0 %on black (turn right - assuming following outside right of the line)
            leftMotor.Speed = 20 + (abs(xn)); % speed up left wheel
            rightMotor.Speed = 20 - (abs(xn));
            count = 0; % set to zero whenever black tape is visible
            tapeHeading = headingAngle;
        elseif error < 0 %(on white -- turn left) 
            leftMotor.Speed = 20 - (abs(xn)); % slow down left wheel
            rightMotor.Speed = 20 + (abs(xn));
            if intensity > 40
                count = count + 1; % count up whenever not on black 
            end
        end
        if count > 5 % Not sure how we want to determine this yet.
            rightMotor.Speed = 0;
            leftMotor.Speed = 0;
            eOld = 0;
            enOld = 0;
            lastR1 = 0;
            lastR2 = 0;
            resetRotation(leftMotor);                     
            resetRotation(rightMotor);
            wpCount = 1;
            [goalX, goalY] = getMission(X,Y);
            scatter(goalX, goalY, "*red");
            state = 2;
        end
    elseif state == 2 % Search Pipe (Lawnmower)
        distance = getDistance(goalX(wpCount), goalY(wpCount), X, Y);
        angleDifference = getAngle(X, Y, headingAngle, goalX(wpCount), goalY(wpCount));
        if distance < 0.1
            rightMotor.Speed = 0;
            leftMotor.Speed = 0;
            wpCount = wpCount+1;
        elseif(abs(angleDifference) > angle_tolerance)
            if angleDifference >= 0
                w = wMax;
            else 
                w = -wMax;
            end
            e = angleDifference;
            xn = (kpT*e) + (kdT*(e-eOld)/ts);      % Command Signal
            if (xn*wStandard > wMax)   % Saturation Limit for PD Controller
                leftMotor.Speed = wMax;  
            elseif (xn*wStandard < wMin)
                leftMotor.Speed = wMin;
            else
                leftMotor.Speed = xn*wStandard;  
            end
        elseif(distance > 0.1)
            en = distance;
            xn = (kpD*en) + (kdD*(en-enOld)/ts);      % Command Signal

            if (xn*vStandard > vMax)   % Saturation Limit for PD Controller
                leftMotor.Speed = vMax;  
            elseif (xn*vStandard < vMin)
                    leftMotor.Speed = vMin;
            else
                leftMotor.Speed = xn*vStandard;
            end
            rightMotor.Speed = leftMotor.Speed;
            enOld = en;
        end
    elseif state == 3
        error = tapeHeading - headingAngle;
        
    end
    
    % Update position
    wl = (double(r1-lastR1)/(360*ts))*2.0*pi;  %rad/second 
    wr = (double(r2-lastR2)/(360.0*ts))*2.0*pi;  % rad/second 
    turningRate = radius*(wr-wl)/L;    %Vehicle Turning Rate rad/s 
    headingAngle = mod(headingAngle + (turningRate*ts), 180);
    vlinear = radius*(wr+wl)/2;
    X = double(X + (cos(headingAngle)*vlinear*ts)); 
    Y = double(Y + (sin(headingAngle)*vlinear*ts));
    
    lastR1 = r1;
    lastR2 = r2;
    i = i + 1;
    XHistory(i) = X;
    YHistory(i) = Y;
    
    pause(ts);                         % Wait for next sampling period
end

scatter(XHistory, YHistory,'.black');
scatter(goalX, goalY, '*red');