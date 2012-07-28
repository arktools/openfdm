within F16;
model NullEngine
    input Real throttle;
    input Real mach;
    input Real alt; 
    output Real thrust;
equation
    thrust = 20000;
end NullEngine;
