%
% STOPSCRIPT  -- Postprocessing of ATRIAS Model Output
%
%

% Simulation Performance
% ----------------------

% get current time
a=toc;

% output simulation performace in % realtime
if(isempty(tout)) 
    fprintf('\n> Simulation failed to run!');
else
    fprintf('\n> Simulation ran %0.1f seconds (%0.0f%% real time performance).\n', a, tout(end)/a*100)
end
      
% Mechanical Limit Collision Flags
% --------------------------------
      
fprintf('\n> Mechanical Limit Collision Flags:')
if any(R_collision_flags)
  fprintf('\n     Right Leg: Shin_Max=%d Shin_Min=%d Thigh_Max=%d Thigh_Min=%d  Leg Extension=%d Leg Flexion=%d', R_collision_flags);
end
if any(L_collision_flags)
  fprintf('\n     Left  Leg: Shin_Max=%d Shin_Min=%d Thigh_Max=%d Thigh_Min=%d  Leg Extension=%d Leg Flexion=%d', L_collision_flags);
end
fprintf('\n\n')