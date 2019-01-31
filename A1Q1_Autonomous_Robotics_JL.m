%% Assignment 1 Question 1 %%
clear all;
close all;

%% Set up 
clear mylego
mylego = legoev3('usb');          

%Motors
%Left Motor %Right Motor
leftMotor = motor(mylego, 'A');      
rightMotor = motor(mylego, 'D');     
resetRotation(leftMotor);            
resetRotation(rightMotor);
leftMotor.Speed = 0;                 
rightMotor.Speed = 0; 
start(leftMotor);                    
start(rightMotor);

lastR1 = 0;
lastR2 = 0;

%Colour Sensor
mycolorsensor = colorSensor(mylego);        % Setup colour sensor

%% Global variables: State and Vehicle Description

PI = 3.14159;
radius = 0.0558/2.0; % wheel radius in meters
circumference = 2*PI*radius; %wheel circumference
L = 0.136; %Distance between wheels (Differential drive) in meters

X = 0.0;                                       %robot location X
Y = 0.0;                                       %robot location Y
headingAngle = 0.0;                            %robot Heading

goalX = 0.0;                                   %goal location X
goalY = 0.0;                                   %goal location Y
goalAngle = 0.0;                               %goal heading angle

%PD Driving Paramaters
distTol = 0.1;                               %meters
vMax = 60;                                   % Motor speed
vMin = 15;
vStandard = 40;
kpD = 0.01;                                  % P controller parameter: DRIVING
kdD = 0.02;                                  % D controller parameter: DRIVING
enOld = 0;
en = 0;

%PD Turning Paramaters
wMax = 30;                                   % Motor speed
wMin = 10;
wStandard = 10;
kpT = 0.1;                                   % P controller parameter: TURNING
kdT = 0.02;                                  % D controller parameter: TURNING
eOld = 0;
e = 0;

count = 0;

%% Tuning Parameters
floor_intensity_level = 35;

intensity_threshold = 40;
intensity_kp = 0.7;

angle_tollerance = 0.25;

%% Operations
state = 1; % begin in state 1
ts = 0.06;% Sampling period (play with this)

while 1 % Continuous Loop (always true)
    
    while state == 1 % Follow Pipe (Black tape) (proportional controller)
        
        intensity = readLightIntensity(mycolorsensor,'reflected');
        
        % Read rotation counter in degrees
        r1 = readRotation(leftMotor)            
        r2 = readRotation(rightMotor) 
        
        desired = floor_intensity_level;
        intensity_error = desired - intensity;
        xn = intensity_error * intensity_kp; 
        
        %if on black tape
        if intensity_error >= 0 %on black (turn right - assuming following outside right of the line)
            leftMotor.Speed = 15 + (abs(xn)); % speed up left wheel
            rightMotor.Speed = 15 - (abs(xn));
            
            count = 0; % set to zero whenever black tape is visible
       
        %else if on while -- turn leff
        elseif intensity_error < 0
            leftMotor.Speed = 15 - (abs(xn)); % slow down left wheel
            rightMotor.Speed = 15 + (abs(xn)); %speed up right wheel
            
            %if not on black
            if intensity > intensity_threshold
                count = count + 1;
            end
        end
        
        %STATE UPDATE
        wl = double(r1-lastR1)/(360.0*ts)*2.0*PI  %rad/second 
        wr = double(r2-lastR2)/(360.0*ts)*2.0*PI  % rad/second 
        
        turningRate = double(radius*(wr-wl)/L);      %Vehicle Turning Rate rad/s 
        headingAngle = wrapTo2Pi(headingAngle + (turningRate*ts));%%%NEW
        
        vlinear = double(radius*(wr+wl)/2)
        
        X = double(X + (cos(headingAngle)*vlinear*ts)) %update X 
        Y = double(Y + (sin(headingAngle)*vlinear*ts)) %update y
        
        lastR1 = r1;
        lastR2 = r2;
        
        if count > 10 % Not sure how we want to determine this yet.
            rightMotor.Speed = 0;
            leftMotor.Speed = 0;
            
            state = 2; % Switch to search behaviour
            beep(mylego, 0.5);
            beep(mylego, 0.5);
        end
        
        pause(ts);
    end
    
    if state == 2
        % Reset Controller Variables
        eOld = 0;
        enOld = 0;
        wpCount = 1;
        
        [goalX, goalY] = getMission(X,Y)
    end
    
    while state == 2 % Search Pipe (Lawnmower)
        
        intensity = readLightIntensity(mycolorsensor,'reflected');
        desired = floor_intensity_level;
        intensity_error = desired - intensity;
        
        %If found black, state = 3
        if intensity_error >= 0
            state = 3; % Switch to search behaviour
            beep(mylego, 0.5);
            beep(mylego, 0.5);
            beep(mylego, 0.5);
            break;
        end
        
        % Read rotation counter in degrees
        r1 = readRotation(leftMotor)           
        r2 = readRotation(rightMotor)    
        
        distance = getDistance(goalX(wpCount), goalY(wpCount), X, Y)
        angleDifference = getAngle(X, Y, headingAngle, goalX(wpCount), goalY(wpCount))
        
        if(distance < 0.1)
             rightMotor.Speed = 0;
             leftMotor.Speed = 0;
             wpCount = wpCount+1 
            
        elseif(abs(angleDifference) > angle_tollerance)
            
            if angleDifference >= 0
                w = wMax;
            else 
                w = -wMax;
            end
            
            e = angleDifference;
            xn = (kpT*e) + (kdT*(e-eOld)/ts);      % Command Signal
            
            if (xn*wStandard > vMax)               % Saturation Limit for PD Controller
                leftMotor.Speed = wMax;  
            
            elseif (xn*wStandard < vMin)
                leftMotor.Speed = wMin;
            
            else
                leftMotor.Speed = xn*wStandard;  
            end
            
            rightMotor.Speed = -leftMotor.Speed;
            
            %STATE UPDATE
            wl = (double(r1-lastR1)/(360.0*ts))*2.0*PI %rad/second 
            wr = (double(r2-lastR2)/(360.0*ts))*2.0*PI  % rad/second 
            
            turningRate = double(radius*(wr-wl)/L)   %Vehicle Turning Rate rad/s 
            headingAngle = wrapto2Pi(headingAngle + (turningRate*ts)); %%%NEW
            
            lastR1 = r1;
            lastR2 = r2;
            
        elseif(distance > 0.1)
            
            en = distance;                      
            xn = (kpD*en) + (kdD*(en-enOld)/ts);      % Command Signal

            if (xn*vStandard > vMax) % Saturation Limit for PD Controller
                leftMotor.Speed = vMax;  
            
            elseif (xn*vStandard < vMin)
                leftMotor.Speed = vMin;

            else
                leftMotor.Speed = xn*vStandard;  
            
            end
            
            rightMotor.Speed = leftMotor.Speed;  

            %STATE UPDATE
            wl = double(r1-lastR1)/(360.0*ts)*2.0*PI  %rad/second 
            wr = double(r2-lastR2)/(360.0*ts)*2.0*PI  % rad/second 
            
            turningRate = double(radius*(wr-wl)/L);   %Vehicle Turning Rate rad/s 
            headingAngle = wrapTo2Pi(headingAngle + (turningRate*ts))%%%NEW
            
            lastR1 = r1;
            lastR2 = r2;
            
            vlinear = double(radius*(wr+wl)/2)

            X = double(X + (cos(headingAngle)*vlinear*ts)) %update X 
            Y = double(Y + (sin(headingAngle)*vlinear*ts)) %update y

            enOld = en;
        end

        pause(ts); %Wait for next sampling period
    end

    
    while state == 3 %Turn around
        pause(ts);  
    end
    
    
    pause(ts); %Wait for next sampling period
end

        
   







