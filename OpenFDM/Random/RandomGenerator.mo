within OpenFDM.Aerodynamics;

connector PortNumber  
        Real n; 
end PortNumber;

function mod  
         input Real x, y;
         output Real z; 
algorithm  
         z := x - div(x, y) * y;
end mod; 

model parameters  
// the numbers of random values: 
       parameter Integer n=100; 
// sample period: 
        parameter Real dt = 1; 
// start time moment: 
        parameter Real start = 1; 
// the index of arrays used in simulation: 
       Integer j; 
algorithm         
        j := integer(time/dt) + 1; 
end parameters;

model UNG   //uniform_number_generator 
                 extends parameters; 
       constant Integer m =  2^31 - 1; 
       constant Integer a= 7^5; 
       constant Integer c=10; 
       Real xmax,  x[n]; 
       Integer j; 
       PortNumber OUT; 
algorithm  
        x[1] := 1.0; 
        for k in 1:n - 1 loop
                x[k + 1] := mod(a*x[k] + c, m); 
        end for; 
        xmax := max(x); 
for k in 1:n loop  // normalization 
                 x[k] := x[k] / xmax; 
         end for; 
         OUT.n := x[j]; 
end UNG;

model NNG   // normal_number_generator
        extends UNG_MULTI; 
        PortNumber OUT; 
        Real x[n];  
algorithm 
        x[j] := OUT.n; 
OUT.n:=sqrt(-2*log(OUT1.n))*sin(6.28*OUT2.n); 
end NNG; 

model white_noise  
       extends parameters; 
       parameter Real sigma = 1; 
       Real xw[n], xa[n];
       PortNumber OUT; 
       NNG nng; 
algorithm  
        xw[j] := sqrt(dt)* nng.OUT.n; 
        xa[j] := sigma*(xw[j] - xw[j-1]) / dt; 
        OUT.n := xa[j];
end white_noise; 