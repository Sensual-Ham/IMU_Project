close all; clc;
%% Create Serial Object
s = serial('COM3');
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

for i = 1:100          %Take this many measurements
    for j = 1:6         %cycle through all reported values
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
end
fprintf('\n');
        
%% exit
fclose all;
delete(instrfindall);
disp('Serial Link Closed')