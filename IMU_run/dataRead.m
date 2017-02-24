close all; clc; clear;
%% Create Serial Object
s = serial('COM22');
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'Parity','none');
fopen(s);

%% Wait for communication link
%call and response
a = 'b';
while a~='a'
    a=fread(s,1,'uchar');   %wait for call from arduino
end
disp('Serial Link Created')
fprintf(s,'%c','a');        %send response

%% Get the datas
meas = 300;
ret_vals = 5;
data = zeros(meas,ret_vals);

for i = 1:meas          %Take this many measurements
    for j = 1:ret_vals         %cycle through all reported values
        for k = 1:20    %read a single value letter by letter
            inchar = fread(s,1,'uchar');
            if (inchar == ' ') || (inchar==9) % space or tab delimiters
                break
            end
            if k == 1
                instring = inchar;
            else
                instring = strcat(instring,inchar);
            end
        end
        data(i,j) = str2double(instring);
    end
    fread(s,1,'uchar');
    if mod(i,10)==0
        fprintf('.'); % display progress
    end
    if mod(i,100)==0
        disp(i); % display progress
    end
end
fprintf('\n');
        
%% exit
fclose all;
delete(instrfindall);
disp('Serial Link Closed')

%% Plot stuff
figure()
hold on
plot((data(:,1)-data(1,1))/1000,data(:,2))
plot((data(:,1)-data(1,1))/1000,data(:,3))
plot((data(:,1)-data(1,1))/1000,data(:,4))
plot((data(:,1)-data(1,1))/1000,data(:,5))
legend('Raw Accel','Filtered Accel','Integrated Gyro','Kalman');


