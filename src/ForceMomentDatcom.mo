model ForceMomentDatcom
    Real s = 1;
    Real Cl = .01;
    Real q = 1;
    Real L;
equation
    L = Cl*q*s;
end ForceMomentDatcom;
