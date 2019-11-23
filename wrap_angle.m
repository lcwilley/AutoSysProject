function angle = wrap_angle(angle)
    angle = angle - 2*pi * floor(angle/(2*pi) + 0.5);
end