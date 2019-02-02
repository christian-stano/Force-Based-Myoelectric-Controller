%% PID Control Implementation 
%ARK 
%PID TEST CODE 1 

% %% Step function input 0 to 1 with two cycles 
% 
% %Step1 
% arr = zeros(1,20); %twenty  time points of 0 before step 
% time = 0:19; 
% arr = [arr, 1]; 
% time = [time, 19.02];
% arr = [arr, ones(1,20)]; % step up to 1 and maintain for 20 time points 
% time = [time, 20:39];
% 
% %Step 2 
% arr = [arr,zeros(1,20)]; 
% time = [time, 40:59]; 
% arr = [arr, 1]; 
% time = [time, 59.02];
% arr = [arr, ones(1,20)]; 
% time = [time, 60:79];
% %Plot input factors 
% subplot(1,2,2)
% plot(time, arr)
% title('Input to PID') 
% xlabel('time') 

%% Step function input 0 to 1 and then 0 to 2  

%Step1 
arr = zeros(1,20); %twenty  time points of 0 before step 
time = 0:19; 
arr = [arr, 1]; 
time = [time, 19.02];
arr = [arr, ones(1,20)]; % step up to 1 and maintain for 20 time points 
time = [time, 20:39];

%Step 2 
arr = [arr,zeros(1,20)]; 
time = [time, 40:59]; 
arr = [arr, 2]; 
time = [time, 59.02];
arr = [arr,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]; 
time = [time, 60:79];

%Plot input array 
subplot(1,2,2)
plot(time, arr)
% title('Input to PID') 
% xlabel('time') 

%%  Sine Wave input 

in = [0:0.1:4*pi];
time = [0:125]; 
arr = sin(in); 
subplot(1,2,2)
plot(time, arr) 
title('Input to PID') 
xlabel('time') 
%% PID loop w/ Step Function 
micro_out = 0; %Theoretical final output from microcontroller 
error =0; % Error from setpoint to input at specific time point 
previousE=0; % error from previous window 
to_plot = []; %intialize for plot view 
threshold = 0.5; % Set-point 
k=1; % Proportional Constant 
k2=1; % Integral Constant 
k3=1; % Derivative Constant 
for i=1:length(arr) % to the length of the  input 
    
    %%(the length would be infinite, so would have to just establish while
    %%loop until power off or some outside command is established. 
    
    %function with feedback from previous output
    input_to_pid = arr(i) - micro_out; 
    
    %Error definition as Threshold (0 Contraction) vs. Input with feedback 
    error = threshold - input_to_pid;
    
    %Proportional Term 
    pe = k * error; 
    
    %Integral Term 
    error_n = error + previousE; %Sum the error from previous window and the current error 
    ie = k2 * i;
    
    %Differential Term 
    d = error - previousE; %Take the current error minus the previous error assuming it is converging in the next window of time
    de = k3 * d; 
    
    %Establish previous error for next window 
    previousE = error;
    
    %Micro output is the sum of all the error terms 
    micro_out = input_to_pid + pe + ie + de;
    
    %Output array for plotting 
    to_plot = [to_plot, micro_out];
    
end 
subplot(1,2,1)
plot(time, to_plot)
title('Output aftere PID')
xlabel('time')

