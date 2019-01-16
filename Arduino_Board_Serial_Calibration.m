function Arduino_Board_Serial_Calibration(block)
% Level-2 MATLAB file S-Function for continuous time variable step demo.
%   Copyright 1990-2009 The MathWorks, Inc.
 
setup(block);
 

function setup(block)
  
    %% Register number of input and output ports
    block.NumInputPorts  = 4;
    block.NumOutputPorts = 0;


    %% Setup functional port properties to dynamically
    %% inherited.
    %block.SetPreCompOutPortInfoToDynamic;
 
    %% Setup functional port to default
    block.SetPreCompPortInfoToDefaults;
  
    %% Setup input ports  
    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DirectFeedthrough = false;
    block.InputPort(1).DatatypeID = 0; % Real
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).SamplingMode   = 'Sample';
    
    block.InputPort(2).Dimensions = 1;
    block.InputPort(2).DirectFeedthrough = false;
    block.InputPort(2).DatatypeID = 0; % Real
    block.InputPort(2).Complexity = 'Real';
    block.InputPort(2).SamplingMode   = 'Sample';
    
    block.InputPort(3).Dimensions = 1;
    block.InputPort(3).DirectFeedthrough = false;
    block.InputPort(3).DatatypeID = 0; % Real
    block.InputPort(3).Complexity = 'Real';
    block.InputPort(3).SamplingMode   = 'Sample';
    
    block.InputPort(4).Dimensions = 1;
    block.InputPort(4).DirectFeedthrough = false;
    block.InputPort(4).DatatypeID = 0; % Real
    block.InputPort(4).Complexity = 'Real';
    block.InputPort(4).SamplingMode   = 'Sample';
%   
% %     block.InputPort(5).Dimensions = 1;
% %     block.InputPort(5).DirectFeedthrough = false;
% %     block.InputPort(5).DatatypeID = 0; % Real
% %     block.InputPort(5).Complexity = 'Real';
% %     block.InputPort(5).SamplingMode   = 'Sample';
%     
%     %% Setup output ports  
%     block.OutputPort(1).Dimensions = 1;
%     block.OutputPort(1).DatatypeID = 0; % double
%     block.OutputPort(1).Complexity = 'Real';
%     
%     block.OutputPort(2).Dimensions = 1;
%     block.OutputPort(2).DatatypeID = 0; % double
%     block.OutputPort(2).Complexity = 'Real';
%      
%     block.OutputPort(3).Dimensions = 1;
%     block.OutputPort(3).DatatypeID = 0; % double
%     block.OutputPort(3).Complexity = 'Real';
%     
%     block.OutputPort(4).Dimensions = 1;
%     block.OutputPort(4).DatatypeID = 0; % double
%     block.OutputPort(4).Complexity = 'Real';
%  
%     block.OutputPort(5).Dimensions = 1;
%     block.OutputPort(5).DatatypeID = 0; % double
%     block.OutputPort(5).Complexity = 'Real';
%     
%     block.OutputPort(6).Dimensions = 1;
%     block.OutputPort(6).DatatypeID = 0; % double
%     block.OutputPort(6).Complexity = 'Real';
%     
%     block.OutputPort(7).Dimensions = 1;
%     block.OutputPort(7).DatatypeID = 0; % double
%     block.OutputPort(7).Complexity = 'Real';
% %     
%     block.OutputPort(8).Dimensions = 1;
%     block.OutputPort(8).DatatypeID = 0; % double
%     block.OutputPort(8).Complexity = 'Real';
%     
%     block.OutputPort(9).Dimensions = 1;
%     block.OutputPort(9).DatatypeID = 0; % double
%     block.OutputPort(9).Complexity = 'Real';

  
  %% Set block sample time to variable sample time
    %block.SampleTimes = [0.05 0];
    block.SampleTimes = [.01 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';

  
  %% Register methods
    %block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions',    @InitConditionmyArduinoSerial); 
    block.RegBlockMethod('Terminate',               @TerminateArduino);
    %block.RegBlockMethod('Derivatives',             @ArduinoCommunication); 
    block.RegBlockMethod('Update',                 @Output);
    %block.RegBlockMethod('Update',                  @Update); 
    

function InitConditionmyArduinoSerial(block)
    
    % Global variable
    global myArduinoSerial;
    %% Create Arduino Serial Connection Object for COM4
    % BT '/dev/cu.H-C-2010-06-01-SPPDev';
    % '/dev/cu.usbmodem1461'
    myArduinoSerial = serial('/dev/cu.usbmodem14311','BaudRate',115200) 


    %% Open Arduino Serial Connection
    fopen(myArduinoSerial); % initiate arduino communication


function Output(block) 
    % Global variable
    global myArduinoSerial;
  
    
    try 
      %% Send to Arduino
        % Read Input-Values from Block
        speed1 = num2str(floor(block.InputPort(1).Data) + 500);
        speed2 = num2str(floor(block.InputPort(2).Data) + 500);
        speed3 = num2str(floor(block.InputPort(3).Data) + 800);
        speed4 = num2str(floor(block.InputPort(4).Data) + 500);
        
        
%         Angle_1_des = num2str(floor(block.InputPort(5).Data) + 3000);
%         Angle_2_des = num2str(floor(block.InputPort(6).Data) + 3000);
%         Angle_3_des = num2str(floor(block.InputPort(7).Data) + 3000);
%         Angle_4_des = num2str(floor(block.InputPort(8).Data) + 3000);
%         Linear_des = num2str(floor(block.InputPort(9).Data) + 3000);
          
        
        % Create Send Command
       % arduinoSendData = ['M' Speed_1_des ';' Speed_2_des ';' Speed_3_des ';' Speed_4_des ';' Angle_1_des ';' Angle_2_des ';' Angle_3_des ';' Angle_4_des ';' Linear_des '#'];
        %arduinoSendData = ['M' speed2 ';' speed3 '#']
        arduinoSendData = [speed2 speed3 '#']
       
        % Send Commandstring to Arduino
        fprintf(myArduinoSerial,'%s',arduinoSendData)
%         
%         Read from Arduino
%       Read response from Arduino
        arduinoReceivedData = fgetl(myArduinoSerial);
% 
%         % Check if received Command-String is valid
%         % String Not Empty && Identification is existing
%         if ~isempty(arduinoReceivedData) && strcmp(arduinoReceivedData(1), 'A')
%             
%             % Split string into Substrings
%             tmpArduinoReceivedData = strsplit(arduinoReceivedData(2:length(arduinoReceivedData)),';');
%         
%             % Save Data
%             qx = (str2double(tmpArduinoReceivedData(1))-5000)/1000;
%             qy = (str2double(tmpArduinoReceivedData(2))-5000)/1000;
%             qz = (str2double(tmpArduinoReceivedData(3))-5000)/1000;
%             qw = (str2double(tmpArduinoReceivedData(4))-5000)/1000;
%             
%             ex = str2double(tmpArduinoReceivedData(5))-2000;
%             ey = str2double(tmpArduinoReceivedData(6))-2000;
%             ez = str2double(tmpArduinoReceivedData(7))-2000;
%             
%                  
%             block.OutputPort(1).Data = qx;
%             block.OutputPort(2).Data = qy;
%             block.OutputPort(3).Data = qz;
%             block.OutputPort(4).Data = qw;
%             block.OutputPort(5).Data = ex;
%             block.OutputPort(6).Data = ey;
%             block.OutputPort(7).Data = ez;
%        
%         end
% %     
        
    catch exception

        getReport(exception,'extended','hyperlinks','ON')

    end

 function TerminateArduino(block)
     
    global myArduinoSerial;
         
   % arduinoSendData = ['M' '3000' ';' '3000' ';' '3000' ';' '3000' ';' '3000' ';' '3000' ';' '3000' ';' '3000' ';' '3000' '#'];
    %arduinoSendData = ['M' '5000' ';' '5000' ';' '5000' ';' '5000' '#'];
     %arduinoSendData = ['M' '5000' ';' '5000' '#'];  
     arduinoSendData = ['500' '800' '#'];
    % Send Commandstring to Arduion
    fprintf(myArduinoSerial,'%s',arduinoSendData)
  
    delete(instrfindall)

 
  