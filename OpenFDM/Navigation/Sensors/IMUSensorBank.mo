within OpenFDM.Navigation.Sensors;

model IMUSensorBank
    input Real[3] w_real(start=0);
    discrete Real[3] w_meas(start=0);
    //Real[3] seed(start={27,10089,61});
    parameter Real samplePeriod=0.01;
protected
    Sensor[3] gyro;
equation
    connect(w_real[:],gyro[:].real);
    connect(w_meas[:],gyro[:].meas);
end IMUSensorBank;
