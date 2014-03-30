function [angleVsTime] = QuadSim(filename,kp)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	QuadSim
%
%	Author: Brandon Riches
%	Date  : August 2013
%
%	This function simulates the state of a '+' configuration Quadrocopter
%	as it evolves in time, taking input such as mass, moment of inertia
%	tensor, thrust from motors, battery capacity, etc.
%
%	It takes as input a specially formatted .txt file ("filename"), stored 
%	in the same directory.
%
%	File format must be as follows:
%
%	Test ID: (ID Number)
%	blank line
%	Total Mass: (mass in kg)
%	Moments of Inertia: [Ixx, Iyy, Izz]
%	Thrust: (Current proportional factor in newtons / amp)
%	Current draw: [max, min] (in amps)
%	Battery Capacity: (mAh)
%	Rotational DOF axis: [x, y, z] (boolean)
%	Translational DOF axis: [x, y, z] (boolean)
%
%
%	END FILE FORMAT.
%
%	Motors 1 and 4 operate on the x axis, with motor one on the +ve x.
%	Motors 2 and 3 operate on the y axis, with motor two on the +ve y.
%									    
%										Y
%
%									    M2
%									    |
%									    |
%						X		M1------Z------M4
%									    |
%									    |
%									    M3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set the default filename
if nargin == 0, filename = 'Test1_y_rotation.txt'; end
angleVsTime = 0;

clc; disp('********************QuadSim********************');

%File open and read
TXT_ID = fopen(filename,'r+'); %Open the file
TXT = fread(TXT_ID,'char'); %Read the file into MATLAB
fclose(TXT_ID); %Close the file
%fprintf('%s',TXT); %Print the read file to the screen

%Create the structure which test specifications will be stored into
Quad = struct('TestID', 0, ...
			  'MomentInertia', [0,0,0], ...
			  'Thrust', 0, ...
			  'CurrentDraw', [0,0], ...
			  'Battery', 0, ...
			  'RDOFA', [0,0,0], ...
			  'TDofA', [0,0,0]);
		  
%Parse data from the .TXT file
Quad = ParseTXT(TXT,Quad);

%Initialize State Variables and general constants
MOTOR_PWM_MAX	= 1600; %Duration of HIGH pulse in motor signal, microseconds
MOTOR_PWM_MIN	= 900;

MOTOR_START		= 1350;  %Initialize motors to this speed

M1S				= MOTOR_START;	%Speed of the four motors, given in microseconds. 
M2S				= MOTOR_START;	%The speed references the input signal to the speed
M3S				= MOTOR_START;	%controllers, and has no direct relation to the rpm
M4S				= MOTOR_START;

GYRO_NOISE		= 0;	%Gain for the noise level in gyro signal
ACCEL_NOISE		= 0;	%Gain for noise level in accel signal

ARM_LENGTH		= 0.25; %Length of the quadcopter arms in meters.

SENSOR_FREQ		= 100; % Frequency of update for sensor data.
dt				= 0.001; % dt used for simulation

phi				= 0; %Angle between x and horizontal. If +X is above horizon, phi is +ve
theta			= 0; %Angle between y and horizontal. If +Y is above horizon, theta is +ve
gamma			= 0; %Angle between z and starting orientation. If +Z has moved ccw, gamma is +ve

AX_a			= 0; %Physical value for acceleration along x-axis
AY_a			= 0; %Physical value for acceleration along y-axis
AZ_a			= 0; %Physical value for acceleration along z-axis

WX_a			= 0; %Physical value for angular velocity around x-axis
WY_a			= 0; %Physical value for angular velocity around y-axis
WZ_a			= 0; %Physical value for angular velocity around z-axis

%Initialize sensor data simulation
% '_s' suffix indicates sensor data, not actual value
AX_s = zeros(1,10);
AY_s = zeros(1,10);
AZ_s = zeros(1,10);

WX_s = zeros(1,10);
WY_s = zeros(1,10);
WZ_s = zeros(1,10);


%%%%%%%%%%%%%
% Main Loop.%

rotate_x = Quad.RDOFA(1);
rotate_y = Quad.RDOFA(2);
rotate_z = Quad.RDOFA(3);
translate_x = Quad.TDOFA(1);
translate_y = Quad.TDOFA(2);
translate_z = Quad.TDOFA(3);

%Set some infinitesimal initial offset. 
[phi,theta,gamma] = rand_offset(phi, theta, gamma);
phi = 1;
SIM_ITERATIONS = 100000;

ki = 0.19396;
kd = 0.78829;
kp = 0.84569;
Iterm = 0;

angleVsTime = zeros(SIM_ITERATIONS,1);
prev_time = 0;
last_error = 0;

%Set the timer running. Depending on what dt is, this sets the duration of
%the simulation.
for t = 0:SIM_ITERATIONS

        time = (t-1)*dt;					 
        %Check for sim-allowed rotation around y-axis.
        %Note: this case is analogous to our test apparatus and it's all im
        %coding for now. 
        
        angleVsTime(t+1) = phi;

        %Check the update time of PID
        %Do PID update. 
        if (time-prev_time) >= 0.01
            
            last_error = phi;
            
            P = kp * phi;
            Iterm = Iterm + ki * phi * 0.01;
            D = kd * (phi - last_error) / 0.01;

            C = P + Iterm + D;
            
            M1S = 1350 - C;
            M4S = 1350 + C;
           
            prev_time = time;
            
        end

        %Calculate the force in newtons each motor is applying by mapping the
        %speed in microseconds to the current range, and then multplying by the
        %thrust constant specified in the .txt (given in N/A)
        
        FX_P = Quad.Thrust * M1S; %Motor on positive end of x axis

        FX_N = Quad.Thrust * M4S; %Motor on negative end of x axis

        FY_P = Quad.Thrust * ( ... 
                            ( M2S - MOTOR_PWM_MIN ) * ( Quad.CurrentDraw(2)-Quad.CurrentDraw(1) )/( MOTOR_PWM_MAX - MOTOR_PWM_MIN ) + Quad.CurrentDraw(1) ...
                             ); %Motor on positive end of y axis

        FY_N = Quad.Thrust * ( ... 
                            ( M3S - MOTOR_PWM_MIN ) * ( Quad.CurrentDraw(2)-Quad.CurrentDraw(1) )/( MOTOR_PWM_MAX - MOTOR_PWM_MIN ) + Quad.CurrentDraw(1) ...
                             ); %Motor on negative end of y axis

        %Calculate the torque, angular acceleration, angular velocity, and
        %increase in angle
        MOTOR_TORQUE_Y = FX_P * ARM_LENGTH - FX_N * ARM_LENGTH;	

        %Integrate values
        alphaR = MOTOR_TORQUE_Y / 0.02054;

        WY_a = WY_a + alphaR * dt;

        phi = phi + WY_a * dt;
end

plot(angleVsTime);

end
function [phi, theta, gamma] = rand_offset(phi, theta, gamma)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	rand_offset()
%
%		Simple function to add a random initial offset to the quadcopter, 
%		gets things moving. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi = phi + 0.01;
theta = theta + 0.01;
gamma = gamma + 0.01;
end

function [reading] = getSensorData(sensor, axis, phi, theta, gamma, AX_a, AY_a, AZ_a, WX_a, WY_a, WZ_a, GYRO_NOISE, ACCEL_NOISE)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%																		
%	getSensorData
%
%		Input: 
%			Sensor: Sensor selection to generate data point for
%			Axis:	Axis selection		"		"		"
%			Phi,theta,gamma,...: State variables required for the
%								 calculation of the sensor data
%
%		Output:
%			Reading: The sensor data required for the selected axis and
%					 sensor. Contains noise.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% We are simulating data from two sensors at the current version
switch sensor
	
	%The actual values (ex. AX_a) for the acceleration along the axes is calculated
	%here, in the formulas shown in the 3 cases. The sensor data is
	%generated by taking this value and adding noise. 
	%NB: This calculation if valid only for NON-ACCELERATING states. That
	%is to say, it calculates only the acceleration due to gravity. 
	case 'accel'
		if axis == 1; %X_AXIS
			reading = cos(theta*pi/180)*sin(phi*pi/180)*9.81369388;
		end
		if axis == 2; %Y_axis
			reading = sin(theta*pi/180)*9.81369388;
		end
		if axis == 3; %Z_axis
			reading = cos(theta*pi/180)*cos(phi*pi/180)*9.81369388;
		end
		%Calculate sensor reading
		reading = reading + (ACCEL_NOISE) * randn;
		
		
	case 'gyro'
		if axis == 1; %X_AXIS
			reading = WX_a + randn * (GYRO_NOISE);
		end
		if axis == 2; %Y_axis
			reading = WY_a + randn * (GYRO_NOISE);
		end
		if axis == 3; %Z_axis
			reading = WZ_a + randn * (GYRO_NOISE);
		end
		
end
end

function [QuadStruct] = ParseTXT(TXT,QuadStruct)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%																		
%	ParseTXT
%
%		Input: 
%			TXT: A string containing data read from the configuration file
%			QuadStruct: The structure intended to store the config. data
%
%		Output:
%			QuadStruct: The structure containing parsed data from the txt
%						string.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

filepointer = 0; %Specifies the current location of the scanner
eol = 0;
eolbool = false;
fieldstart = 0;
fieldstartbool = false;
openbkt = 0;
openbktbool = false;
closebkt = 0;
closebktbool = true;
comma1 = 0;
comma1bool = false;
comma2 = 0;
comma2bool = false;

ID = '';
Mass = 0; %#ok<NASGU>
MomentInertia = [0,0,0];
Thrust = 0; %#ok<NASGU>
CurrentMM = [0,0];
BatCap = 0; %#ok<NASGU>
RDOFA = [0,0,0];
TDOFA = [0,0,0];


%Scan for ID Number
while (~eolbool) || (~fieldstartbool)
	
	filepointer = filepointer + 1;
		
	%Check for ':' character (indicating field start)
	if TXT(filepointer) == ':'
		fieldstart = filepointer;
		fieldstartbool = true;
	end
	
	%Check for newline char (indicating field end
	if TXT(filepointer) == 10; %Check for newline char
		eol = filepointer;
		eolbool = true;
	end
	
	if filepointer > 20
		disp('Possible .txt formatting error. Check your file!');
	end
end

% ID number located, save it and remove whitespace
for i = fieldstart+1:eol
	ID = cat(2,ID,TXT(i));
end
ID = strtrim(ID);

fieldstartbool = false;
eolbool = false;

%Skip the blank line
filepointer = filepointer + 2;

%Scan for total mass
while (~eolbool) || (~fieldstartbool)
	
	filepointer = filepointer + 1;
		
	%Check for ':' character (indicating field start)
	if TXT(filepointer) == ':'
		fieldstart = filepointer;
		fieldstartbool = true;
	end
	
	%Check for newline char (indicating field end
	if TXT(filepointer) == 10; %Check for newline char
		eol = filepointer;
		eolbool = true;
	end
end
myStr = '';
% Total mass located, save it and remove whitespace
for i = fieldstart+1:eol
	myStr = cat(2,myStr,TXT(i));
end
Mass = str2double(myStr);

fieldstartbool = false;
eolbool = false;

%Scan for moments of inertia
while (~fieldstartbool) || (~eolbool) || (~openbktbool) || (~closebktbool) || (~comma1bool) || (~comma2bool)
	
	filepointer = filepointer + 1;
		
	%Check for ':' character (indicating field start)
	if TXT(filepointer) == ':'
		fieldstart = filepointer;
		fieldstartbool = true;
	end
	
	%Check for '[' character, indicating matrix start
	if TXT(filepointer) == '['
		openbkt = filepointer;
		openbktbool = true;
	end
	
	%Check for ']' character, indicating matrix end
	if TXT(filepointer) == ']'
		closebkt = filepointer;
		closebktbool = true;
	end
	
	%Check for ',' character occurence
	if TXT(filepointer) == ','
		if comma1bool == false
			comma1 = filepointer;
			comma1bool = true;
		end
		if comma1bool == true
				comma2 = filepointer;
				comma2bool = true;
		end
	end
	
	%Check for newline char (indicating field end
	if TXT(filepointer) == 10 %Check for newline char
		eol = filepointer;
		eolbool = true;
	end
end

myStr = '';
%Inertia matrix located, remove unnecessary characters
for i = openbkt+1:comma1-1
	myStr = cat(2,myStr,TXT(i));
end

MomentInertia(1) = str2double(myStr);
myStr = '';

for i = comma1+1:comma2-1
	myStr = cat(2,myStr,TXT(i));
end

MomentInertia(2) = str2double(myStr);
myStr = '';

for i = comma2+1:closebkt-1
	myStr = cat(2,myStr,TXT(i));
end

MomentInertia(3) = str2double(myStr);

myStr = '';

eolbool = false;
fieldstartbool = false;
openbktbool = false;
closebktbool = true;
comma1bool = false;

%Scan for thrust
while (~eolbool) || (~fieldstartbool)
	
	filepointer = filepointer + 1;
		
	%Check for ':' character (indicating field start)
	if TXT(filepointer) == ':'
		fieldstart = filepointer;
		fieldstartbool = true;
	end
	
	%Check for newline char (indicating field end
	if TXT(filepointer) == 10; %Check for newline char
		eol = filepointer;
		eolbool = true;
	end
end

% Thrust located, save it and remove whitespace
for i = fieldstart+1:eol
	myStr = cat(2,myStr,TXT(i));
end
Thrust = str2double(myStr);

fieldstartbool = false;
eolbool = false;

%Scan for Current draw
while (~fieldstartbool) || (~eolbool) || (~openbktbool) || (~closebktbool) || (~comma1bool)
	
	filepointer = filepointer + 1;
		
	%Check for ':' character (indicating field start)
	if TXT(filepointer) == ':'
		fieldstart = filepointer;
		fieldstartbool = true;
	end
	
	%Check for '[' character, indicating matrix start
	if TXT(filepointer) == '['
		openbkt = filepointer;
		openbktbool = true;
	end
	
	%Check for ']' character, indicating matrix end
	if TXT(filepointer) == ']'
		closebkt = filepointer;
		closebktbool = true;
	end
	
	%Check for ',' character occurence
	if TXT(filepointer) == ','
		comma1 = filepointer;
		comma1bool = true;
	end
	
	%Check for newline char (indicating field end
	if TXT(filepointer) == 10 %Check for newline char
		eol = filepointer;
		eolbool = true;
	end
end

myStr = '';

%Current draw array found, isolate the values and store

for i = openbkt+1:comma1-1
	myStr = cat(2,myStr,TXT(i));
end

CurrentMM(1) = str2double(myStr);
myStr = '';

for i = comma1+1:closebkt-1
	myStr = cat(2,myStr,TXT(i));
end

CurrentMM(2) = str2double(myStr);
myStr = '';

eolbool = false;
fieldstartbool = false;
openbktbool = false;
closebktbool = false;
comma1bool = false;
comma2bool = false;

%Scan for battery capacity
while (~eolbool) || (~fieldstartbool)
	
	filepointer = filepointer + 1;
		
	%Check for ':' character (indicating field start)
	if TXT(filepointer) == ':'
		fieldstart = filepointer;
		fieldstartbool = true;
	end
	
	%Check for newline char (indicating field end
	if TXT(filepointer) == 10; %Check for newline char
		eol = filepointer;
		eolbool = true;
	end
end

% Thrust located, save it and remove whitespace
for i = fieldstart+1:eol
	myStr = cat(2,myStr,TXT(i));
end
BatCap = str2double(myStr);


%Scan for RDOFA matrix (boolean)
while  (~openbktbool) || (~closebktbool) || (~comma1bool) || (~comma2bool)
	
	filepointer = filepointer + 1;
	
	%Check for '[' character, indicating matrix start
	if TXT(filepointer) == '['
		openbkt = filepointer;
		openbktbool = true;
	end
	
	%Check for ']' character, indicating matrix end
	if TXT(filepointer) == ']'
		closebkt = filepointer;
		closebktbool = true;
	end
	
	%Check for ',' character occurence
	if TXT(filepointer) == ','
		if comma1bool == false
			comma1 = filepointer;
			comma1bool = true;
		end
		if comma1bool == true
				comma2 = filepointer;
				comma2bool = true;
		end
	end
end

myStr = '';
%RDOFA matrix located, remove unnecessary characters
for i = openbkt+1:comma1-1
	myStr = cat(2,myStr,TXT(i));
end

RDOFA(1) = str2double(myStr);
myStr = '';

for i = comma1+1:comma2-1
	myStr = cat(2,myStr,TXT(i));
end

RDOFA(2) = str2double(myStr);
myStr = '';

for i = comma2+1:closebkt-1
	myStr = cat(2,myStr,TXT(i));
end

RDOFA(3) = str2double(myStr);

openbktbool = false;
closebktbool = false;
comma1bool = false;
comma2bool = false;

%Scan for TDOFA matrix (boolean)
while (~openbktbool) || (~closebktbool) || (~comma1bool) || (~comma2bool)
	
	filepointer = filepointer + 1;
	
	%Check for '[' character, indicating matrix start
	if TXT(filepointer) == '['
		openbkt = filepointer;
		openbktbool = true;
	end
	
	%Check for ']' character, indicating matrix end
	if TXT(filepointer) == ']'
		closebkt = filepointer;
		closebktbool = true;
	end
	
	%Check for ',' character occurence
	if TXT(filepointer) == ','
		if comma1bool == false
			comma1 = filepointer;
			comma1bool = true;
		end
		if comma1bool == true
				comma2 = filepointer;
				comma2bool = true;
		end
	end
end

myStr = '';
%TDOFA matrix located, remove unnecessary characters
for i = openbkt+1:comma1-1
	myStr = cat(2,myStr,TXT(i));
end

TDOFA(1) = str2double(myStr);
myStr = '';

for i = comma1+1:comma2-1
	myStr = cat(2,myStr,TXT(i));
end

TDOFA(2) = str2double(myStr);
myStr = '';

for i = comma2+1:closebkt-1
	myStr = cat(2,myStr,TXT(i));
end

TDOFA(3) = str2double(myStr);

%Now that all values are parsed, pass into the struct
QuadStruct.TestID = str2double(ID);
QuadStruct.Mass = Mass;
QuadStruct.MomentInertia = MomentInertia;
QuadStruct.Thrust = Thrust;
QuadStruct.CurrentDraw = CurrentMM;
QuadStruct.Battery = BatCap;
QuadStruct.RDOFA = RDOFA;
QuadStruct.TDOFA = TDOFA;
end