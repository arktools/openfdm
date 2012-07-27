within OpenFDM;

model Airframe "this is a body shape with a 3-2-1 angle sequence common for aircraft, and with quaternions disabled by default due to numerical problems with the openmodelica solver and quaterions"

  import MB=Modelica.Mechanics.MultiBody;
  import Cv=Modelica.SIunits.Conversions;
  extends MB.Parts.Body(
    useQuaternions=false,
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates);

  /*Real p "roll rate"; */
  /*Real q "pitch rate"; */
  /*Real r "yaw rate"; */
  /*Real roll;*/
  /*Real pitch;*/
  /*Real yaw;*/
  /*Real alpha;*/
  /*Real beta;*/
  /*Real roll_deg;*/
  /*Real pitch_deg;*/
  /*Real yaw_deg;*/
  /*Real vt;*/

  Environment env; //(alpha=alpha,beta=beta,p=p,q=q,r=r,vt=vt);

  Real gD = 9.8; //acceleration

  //parameter Real gamma = 0; // flight path angle
  //parameter Real psiDot = 0; // heading rate

initial equation

  //Real rc_a, rc_b; // rate of climb temp variables
  //Real tc_a, tc_b, tc_c, tc_G; // turn coordination temp variables

  
  // rate of climb constraint
  //pitch = atan((rc_a*rc_b+sin(gamma)*sqrt(rc_a^2-(sin(gamma))^2+rc_b^2))/(rc_a^2-(sin(gamma))^2));

  // turn-coordination constraint
  //roll = atan(tc_G*(cos(beta)/cos(alpha))*((tc_a-tc_b^2)+tc_b*tan(alpha)*sqrt(tc_c*(1-tc_b^2)+tc_G^2*(sin(beta))^2))/(tc_a^2-tc_b^2*(1+tc_c^2*(tan(alpha))^2)));

  //a_0 = {0,0,0};
  //a_0[3] = 0; // no accelerations in down frame, glide slope
  //phi[1] = 0; // don't change yaw angle
  //r_0 = {0,0,-10000}; // don't change position

equation

  /*rc_a = cos(alpha)*cos(beta);*/
  /*rc_b = sin(roll)*cos(beta) + cos(roll)*sin(alpha)*cos(beta);*/
  /*tc_G = psiDot*vt/gD;*/
  /*tc_a = 1-tc_G*tan(alpha)*sin(beta);*/
  /*tc_b = sin(gamma)/cos(beta);*/
  /*tc_c = 1+tc_G^2*(cos(beta))^2;*/

  connect(env.frame,frame_a);

  /*yaw = phi[1];*/
  /*pitch = phi[2];*/
  /*roll = phi[3];*/
  /*yaw_deg = Cv.to_deg(yaw);*/
  /*pitch_deg = Cv.to_deg(pitch);*/
  /*roll_deg = Cv.to_deg(roll);*/
  
end Airframe;

// vim:ts=2:sw=2:expandtab:
