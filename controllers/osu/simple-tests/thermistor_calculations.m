% Given an digital value, find the equivalent voltage
syms digitalValue real
bits = 6;
maxVoltage = 3.3;
voltage = maxVoltage/2^bits * digitalValue;

% Find temperature given a voltage
temperature = ((1.0/( (1.0/298.15) + (1.0/3988.0)*log(4700.0/((3.26/voltage) - 1.0)/10000))) - 273.15);

% Solve for the digital value given a temperature
syms T real
dV = solve(temperature == T,digitalValue);

% Convert from symbolic to matlab function
dV = matlabFunction(dV);