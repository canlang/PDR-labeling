function dang = ang_diff(x,y)
%Calculates difference between the angles
dang = x-y;
dang = wrap2pi(dang+pi)-pi;
end

function angle = wrap2pi(angle)
  pos = angle > 0;  
  angle = mod(angle,2*pi);
  if ((angle == 0) && pos)
      angle = 2*pi;
  end
end