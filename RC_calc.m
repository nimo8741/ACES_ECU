close all
clear all
clc

Vs = 9.9;     % this is the max voltage of the PWM
duty = 0.20;   % this is the duty cycle of the pwm
f_duty = 100;   % this is the frequency of the duty cycle in hz
C = 5*(10^-6);   % this is the capacitance in farads
R = 50000;        % This is the resistance in ohms
t_step = 1/f_duty/100;     % the width of the integration period.  oscillation period/50, in sec

percent = 5;

c_volt = zeros(ceil(1/t_step)*percent,1);      % I will simulate for 1 sec
volt_in = zeros(ceil(1/t_step)*percent,1);     % This is the voltage of the duty cycle

time = zeros(length(c_volt),1);

for i = 1:length(c_volt)
    % first figure out the voltage of the pulse
    if (time(i)/(1/f_duty))-floor(time(i)/(1/f_duty)) > duty
        volt_in(i) = 0;
    else
        volt_in(i) = Vs;
    end
    c_volt(i+1) = c_volt(i) + (volt_in(i)-c_volt(i))/(R*C)*t_step;
    
    
    time(i+1) = time(i) + t_step;
end

%plot(time(1:end-1),volt_in);hold on     %this is to show the PWM
plot(time,c_volt);hold on
line([0,time(end)],[Vs,Vs],'Color','red','LineStyle','--')
line([0,time(end)],[Vs*duty,Vs*duty],'Color','green','LineStyle','--')
legend('Cap Voltage','Max Duty Volt','Desired volt')
xlabel('Time [sec]')
ylabel('Voltage [Volts]')