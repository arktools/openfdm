within F16;

function throttleGearing "computer throttle gearing"
    input Real throttle "engine throttle";    
    output Real throttleGearing "engine throttle gearing";
algorithm
    if (throttle <= 0.77) then  
        throttleGearing := 64.94*throttle;
    else
        throttleGearing := 217.38*throttle-117.38;
    end if;
end throttleGearing;

function rtau "compute reciprocal of time constant for engine"
    input Real dp "power difference";
    output Real rtau "reciprocal of time constant for engine";
algorithm
    if (dp <= 25.0) then
        rtau := 1.0;
    elseif (dp >= 50.0) then
        rtau := 0.1;
    else
        rtau := 1.9-0.36*dp;
    end if;
end rtau;

function powerDerivative "computer derivative of power"
    input Real power "engine power";
    input Real powerCmd "command engine power";
    output Real powerDerivative "derivative of engine power";
protected
    Real p "target power";
    Real t "reciprocal of time constant, 1/T";
algorithm
    if (powerCmd >= 50.0) then
        if (power >= 50.0) then
            p := powerCmd;
            t := 5.0;
        else
            p := 60.0;
            t := rtau(p-power);
        end if;
    else
        if (power >= 50.0 ) then
            p := 40.0;
            t := 5.0;
        else
            p := powerCmd;
            t := rtau(p-power);
        end if;
    end if;
    powerDerivative := t*(p-power);
end powerDerivative;

model Engine
  // input
  input Real throttle;
  input Real alt;
  input Real mach;
  // output
  output Real thrust "engine thrust";
  // state
  Real power "enigne power";
protected
  Real cPow "power coefficient";
algorithm
  cPow := throttleGearing(throttle);
  thrust := computeThrust(power,alt,amach);
equation
  der(power) = powerDerivative(power,cPow);
end Engine;
