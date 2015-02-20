function test_joystick
%TEST_JOYSTICK Test joystick setup with graphical display.

  running = 1;

  % Initialize joystick
  vrj = vrjoystick(1);

  % Create figure and attach close function
  figure('CloseRequestFcn', @onClose);
  
  % Create left stick plot
  subplot(2,2,1);
  hold on; grid on; grid minor; box on; axis square;
  title('Left Stick');
  p1 = plot(0, 0, '*b');
  xlim([-1 1]);
  ylim([-1 1]);

  % Create right stick plot
  subplot(2,2,2);
  hold on; grid on; grid minor; box on; axis square;
  title('Right Stick');
  p2 = plot(0, 0, '*r');
  xlim([-1 1]);
  ylim([-1 1]);
  
  % Create right stick plot
  subplot(2,2,3:4);
  hold on; grid on; grid minor; box on;
  title('Buttons');
  b1 = bar(0, 0);
  ylim([0 1]);
  
  while running
    % Query joystick
    a = axis(vrj);
    b = double(button(vrj));

    % Update the plots
    set(p1, 'XData', a(1), 'YData', -a(2));
    set(p2, 'XData', a(3), 'YData', -a(4));
    set(b1, 'XData', 1:numel(b), 'YData', b);

    % Flush system queue
    drawnow;
  end % while

  % Release joystick
  clear vrj;

  function onClose(src, ~)
  %ONCLOSE Figure close callback function.
  
     running = 0;
     delete(src);
  end % onClose
end % joytest