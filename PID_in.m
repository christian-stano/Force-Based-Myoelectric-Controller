% PID Control Implementation 

% Step function input 
 
arr = zeros(1,20); 
time = 0:19; 
arr = [arr, 1]; 
time = [time, 19.02];
arr = [arr, ones(1,20)]; 
time = [time, 20:39];

arr = [arr,zeros(1,20)]; 
time = [time, 40:59]; 
arr = [arr, 1]; 
time = [time, 59.02];
arr = [arr, ones(1,20)]; 
time = [time, 60:79];
subplot(1,2,2)
plot(time, arr)

%PID loop 
micro_out = 0; 
error =0; 
previousE=0; 
to_plot = [];
threshold = 0.5;
k=1; 
k2=0; 
k3=0; 
for i=1:length(arr) 
    
    %function with feedback from previous output
    input_to_pid = arr(i) - micro_out; 
    
    error = threshold - input_to_pid;
    pe = k * error; 
    
    error_n = error + previousE;
    ie = k2 * i;
    
    d = error - previousE; 
    de = k3 * d; 
    
    error = previousE;
    
    micro_out = input_to_pid + pe + ie + de
    
    to_plot = [to_plot, micro_out]
    
end 
subplot(1,2,1)
plot(time, to_plot)