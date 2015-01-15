% Voltage scalar fitting for the ATXMega128a1
x1 = [3 7 11 15 19 23 27 31 35]; % Digital Value
y1 = [0.497;
     0.929;
     1.227;
     1.457;
     1.646;
     1.841;
     1.981;
     2.126;
     2.259]'; % Voltage
 
% Fit a second order polynomial
p = polyfit(x1,y1,2);
%{
% Plot the polynomial and the raw data
x2 = 3:35;
y2 = p(1)*x2.^2 + p(2)*x2 + p(3);

plot(x1,y1,'o')
hold on
plot(x2,y2)
%}

% Given an digital value, find the equivalent voltage
syms digitalValue real
% bits = 6;
% maxVoltage = 3.3;
% voltage = maxVoltage/2^bits * digitalValue;
voltage = digitalValue^2*p(1) + digitalValue*p(2) + p(3);

% Find temperature given a voltage.  Room temperature voltage is ~2.25V
temperature = ((1.0/( (1.0/298.15) + (1.0/3988.0)*log(4700.0/((3.26/voltage) - 1.0)/10000))) - 273.15);

% Convert from symbolic to matlab function
T = matlabFunction(temperature);