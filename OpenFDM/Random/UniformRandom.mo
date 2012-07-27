within OpenFDM.Random;

function random "Pseudo random number generator" 
    input Real[3] seed;
    output Real x;
    output Real[3] outSeed;
algorithm 
    outSeed[1] := rem((171*seed[1]), 30269);
    outSeed[2] := rem((172*seed[2]), 30307);
    outSeed[3] := rem((170*seed[3]), 30323);
    for i in 1:3 loop 
        if outSeed[i] == 0 then 
            outSeed[i] := 1; 
        end if; 
    end for;

    x := rem((outSeed[1]/30269.0+outSeed[2]/30307.0 + outSeed[3]/30323.0), 1.0);
end random;
