state = tg.OutputLog;
time = tg.TimeLog;
variables = 'rB rA rH lB lA lH rBl rBm rAl rAm lBl lBm lAl lAm rHm lHm bR bY bP drBl drBm drAl drAm dlBl dlBm dlAl dlAm drHm dlHm dbR dbY dbP rB rA rH lB lA lH t t';
controllerName = tg.Application;
logDate = sprintf('%d-',floor(clock));
logDate = logDate(1:end-1);
logName = [logDate '-' controllerName '.mat'];
save(logName,'state','time','variables')