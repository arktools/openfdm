within OpenFDM.Navigation;

model KalmanFilter
    parameter Integer n = 4;
    parameter Integer m = 1;
    parameter Integer r = 2;
    parameter Integer p = 1;

    parameter Real samplePeriod=0.1;

    parameter Real[n,n] A = {{1.0000,0.0100,0.0000,0.0000},{-0.0000,1.0000,0.0100,0.0000},{-0.0012,-0.0024,0.9983,0.0095},{-0.2283,-0.4767,-0.3353,0.9032}};

    //exp({{0,1,0,0},{0,0,1,0},{0,0,0,1},{-24,-50,-35,-10}};
    parameter Real[n,r] B ={{0,0},{0,0},{0,0},{0,0}};
    parameter Real[m,n] C ={{1,0,0,0}};
    parameter Real[m,p] D =identity(1);

    discrete input Real[m,1] y;
    discrete output Real[n,1] x_hat(start={{110},{65},{45},{85}});
    discrete output Real[n,1] x_hatp(start={{110},{65},{45},{85}});
    discrete output Real[n,n] Q(start=100*identity(n));
    discrete output Real[n,n] Qp(start=100*identity(n));

protected
    discrete Real[n,m] K;
    discrete Real[m,m] invTerm;

algorithm
    when sample(0,samplePeriod) then
        x_hat := pre(x_hatp);
        Q := pre(Qp);
        invTerm := Modelica.Math.Matrices.inv(C*pre(Qp)*transpose(C)+D*transpose(D));
        K := A*pre(Qp)*transpose(C)*invTerm;
        Qp := A*pre(Qp)*transpose(A)+B*transpose(B)+K*C*pre(Qp)*transpose(A);
        x_hatp := A*pre(x_hatp)+K*(y-C*pre(x_hatp));
    end when;
end KalmanFilter;
