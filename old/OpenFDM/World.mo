within OpenFDM;

model World "gravity field info, atmospheric conditions
  World coordinate system + gravity field + default animation definition"

  import SI = Modelica.SIunits;
  import Modelica.Mechanics.MultiBody.Types.GravityTypes;
  import Modelica.Mechanics.MultiBody.Types;

    Interfaces.Frame_b frame_b
    "Coordinate system fixed in the origin of the world frame"
                               annotation (Placement(transformation(extent={{84,
            -16},{116,16}}, rotation=0)));

  parameter Boolean enableAnimation=true
    "= true, if animation of all components is enabled";
  parameter Boolean animateWorld=true
    "= true, if world coordinate system shall be visualized" annotation(Dialog(enable=enableAnimation));
  parameter Boolean animateGravity=true
    "= true, if gravity field shall be visualized (acceleration vector or field center)"
                                                                                          annotation(Dialog(enable=enableAnimation));
  parameter Types.AxisLabel label1="x" "Label of horizontal axis in icon";
  parameter Types.AxisLabel label2="y" "Label of vertical axis in icon";
  parameter Types.GravityTypes gravityType=GravityTypes.UniformGravity
    "Type of gravity field"                                                                                                     annotation (Evaluate=true);
  parameter SI.Acceleration g=9.81 "Constant gravity acceleration"
    annotation (Dialog(enable=gravityType == GravityTypes.UniformGravity));
  parameter Types.Axis n={0,-1,0}
    "Direction of gravity resolved in world frame (gravity = g*n/length(n))"
    annotation (Evaluate=true, Dialog(enable=gravityType == Modelica.Mechanics.
          MultiBody.Types.GravityTypes.UniformGravity));
  parameter Real mue(
    unit="m3/s2",
    min=0) = 3.986e14
    "Gravity field constant (default = field constant of earth)"
    annotation (Dialog(enable=gravityType == Types.GravityTypes.PointGravity));
  parameter Boolean driveTrainMechanics3D=true
    "= true, if 3-dim. mechanical effects of Parts.Mounting1D/Rotor1D/BevelGear1D shall be taken into account";

  parameter SI.Distance axisLength=nominalLength/2
    "Length of world axes arrows"
    annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
  parameter SI.Distance axisDiameter=axisLength/defaultFrameDiameterFraction
    "Diameter of world axes arrows"
    annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
  parameter Boolean axisShowLabels=true "= true, if labels shall be shown"
    annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
  input Types.Color axisColor_x=Modelica.Mechanics.MultiBody.Types.Defaults.FrameColor
    "Color of x-arrow"
    annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
  input Types.Color axisColor_y=axisColor_x
    annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));
  input Types.Color axisColor_z=axisColor_x "Color of z-arrow"
    annotation (Dialog(tab="Animation", group="if animateWorld = true", enable=enableAnimation and animateWorld));

  parameter SI.Position gravityArrowTail[3]={0,0,0}
    "Position vector from origin of world frame to arrow tail, resolved in world frame"
    annotation (Dialog(tab="Animation", group=
          "if animateGravity = true and gravityType = UniformGravity",
          enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
  parameter SI.Length gravityArrowLength=axisLength/2 "Length of gravity arrow"
    annotation (Dialog(tab="Animation", group=
          "if animateGravity = true and gravityType = UniformGravity",
          enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
  parameter SI.Diameter gravityArrowDiameter=gravityArrowLength/
      defaultWidthFraction "Diameter of gravity arrow" annotation (Dialog(tab=
          "Animation", group=
          "if animateGravity = true and gravityType = UniformGravity",
          enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
  input Types.Color gravityArrowColor={0,230,0} "Color of gravity arrow"
    annotation (Dialog(tab="Animation", group=
          "if animateGravity = true and gravityType = UniformGravity",
          enable=enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity));
  parameter SI.Diameter gravitySphereDiameter=12742000
    "Diameter of sphere representing gravity center (default = mean diameter of earth)"
    annotation (Dialog(tab="Animation", group=
          "if animateGravity = true and gravityType = PointGravity",
          enable=enableAnimation and animateGravity and gravityType == GravityTypes.PointGravity));
  input Types.Color gravitySphereColor={0,230,0} "Color of gravity sphere"
    annotation (Dialog(tab="Animation", group=
          "if animateGravity = true and gravityType = PointGravity",
          enable=enableAnimation and animateGravity and gravityType == GravityTypes.PointGravity));

  parameter SI.Length nominalLength=1 "\"Nominal\" length of multi-body system"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultAxisLength=nominalLength/5
    "Default for length of a frame axis (but not world frame)"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultJointLength=nominalLength/10
    "Default for the fixed length of a shape representing a joint"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultJointWidth=nominalLength/20
    "Default for the fixed width of a shape representing a joint"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultForceLength=nominalLength/10
    "Default for the fixed length of a shape representing a force (e.g. damper)"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultForceWidth=nominalLength/20
    "Default for the fixed width of a shape represening a force (e.g. spring, bushing)"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultBodyDiameter=nominalLength/9
    "Default for diameter of sphere representing the center of mass of a body"
    annotation (Dialog(tab="Defaults"));
  parameter Real defaultWidthFraction=20
    "Default for shape width as a fraction of shape length (e.g., for Parts.FixedTranslation)"
    annotation (Dialog(tab="Defaults"));
  parameter SI.Length defaultArrowDiameter=nominalLength/40
    "Default for arrow diameter (e.g., of forces, torques, sensors)"
    annotation (Dialog(tab="Defaults"));
  parameter Real defaultFrameDiameterFraction=40
    "Default for arrow diameter of a coordinate system as a fraction of axis length"
    annotation (Dialog(tab="Defaults"));
  parameter Real defaultSpecularCoefficient(min=0) = 0.7
    "Default reflection of ambient light (= 0: light is completely absorbed)"
    annotation (Dialog(tab="Defaults"));
  parameter Real defaultN_to_m(unit="N/m", min=0) = 1000
    "Default scaling of force arrows (length = force/defaultN_to_m)"
    annotation (Dialog(tab="Defaults"));
  parameter Real defaultNm_to_m(unit="N.m/m", min=0) = 1000
    "Default scaling of torque arrows (length = torque/defaultNm_to_m)"
    annotation (Dialog(tab="Defaults"));

  /* The World object can only use the Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape model, but no
     other models in package Modelica.Mechanics.MultiBody.Visualizers, since the other models access
     data of the "outer Modelica.Mechanics.MultiBody.World world" object, i.e., there are
     mutually dependent classes. For this reason, the higher level visualization
     objects cannot be used.
  */
protected
  parameter Integer ndim=if enableAnimation and animateWorld then 1 else 0;
  parameter Integer ndim2=if enableAnimation and animateWorld and
      axisShowLabels then 1 else 0;

  // Parameters to define axes
  parameter SI.Length headLength=min(axisLength, axisDiameter*Types.Defaults.
      FrameHeadLengthFraction);
  parameter SI.Length headWidth=axisDiameter*Types.Defaults.
      FrameHeadWidthFraction;
  parameter SI.Length lineLength=max(0, axisLength - headLength);
  parameter SI.Length lineWidth=axisDiameter;

  // Parameters to define axes labels
  parameter SI.Length scaledLabel=Modelica.Mechanics.MultiBody.Types.Defaults.FrameLabelHeightFraction*
      axisDiameter;
  parameter SI.Length labelStart=1.05*axisLength;

  // x-axis
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowLine(
    shapeType="cylinder",
    length=lineLength,
    width=lineWidth,
    height=lineWidth,
    lengthDirection={1,0,0},
    widthDirection={0,1,0},
    color=axisColor_x,
    specularCoefficient=0) if enableAnimation and animateWorld;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape x_arrowHead(
    shapeType="cone",
    length=headLength,
    width=headWidth,
    height=headWidth,
    lengthDirection={1,0,0},
    widthDirection={0,1,0},
    color=axisColor_x,
    r={lineLength,0,0},
    specularCoefficient=0) if enableAnimation and animateWorld;
  Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines x_label(
    lines=scaledLabel*{[0, 0; 1, 1],[0, 1; 1, 0]},
    diameter=axisDiameter,
    color=axisColor_x,
    r_lines={labelStart,0,0},
    n_x={1,0,0},
    n_y={0,1,0},
    specularCoefficient=0) if enableAnimation and animateWorld and axisShowLabels;

  // y-axis
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowLine(
    shapeType="cylinder",
    length=lineLength,
    width=lineWidth,
    height=lineWidth,
    lengthDirection={0,1,0},
    widthDirection={1,0,0},
    color=axisColor_y,
    specularCoefficient=0) if enableAnimation and animateWorld;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape y_arrowHead(
    shapeType="cone",
    length=headLength,
    width=headWidth,
    height=headWidth,
    lengthDirection={0,1,0},
    widthDirection={1,0,0},
    color=axisColor_y,
    r={0,lineLength,0},
    specularCoefficient=0) if enableAnimation and animateWorld;
  Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines y_label(
    lines=scaledLabel*{[0, 0; 1, 1.5],[0, 1.5; 0.5, 0.75]},
    diameter=axisDiameter,
    color=axisColor_y,
    r_lines={0,labelStart,0},
    n_x={0,1,0},
    n_y={-1,0,0},
    specularCoefficient=0) if enableAnimation and animateWorld and axisShowLabels;

  // z-axis
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowLine(
    shapeType="cylinder",
    length=lineLength,
    width=lineWidth,
    height=lineWidth,
    lengthDirection={0,0,1},
    widthDirection={0,1,0},
    color=axisColor_z,
    specularCoefficient=0) if enableAnimation and animateWorld;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape z_arrowHead(
    shapeType="cone",
    length=headLength,
    width=headWidth,
    height=headWidth,
    lengthDirection={0,0,1},
    widthDirection={0,1,0},
    color=axisColor_z,
    r={0,0,lineLength},
    specularCoefficient=0) if enableAnimation and animateWorld;
  Modelica.Mechanics.MultiBody.Visualizers.Internal.Lines z_label(
    lines=scaledLabel*{[0, 0; 1, 0],[0, 1; 1, 1],[0, 1; 1, 0]},
    diameter=axisDiameter,
    color=axisColor_z,
    r_lines={0,0,labelStart},
    n_x={0,0,1},
    n_y={0,1,0},
    specularCoefficient=0) if enableAnimation and animateWorld and axisShowLabels;

  // Uniform gravity visualization
  parameter SI.Length gravityHeadLength=min(gravityArrowLength,
      gravityArrowDiameter*Types.Defaults.ArrowHeadLengthFraction);
  parameter SI.Length gravityHeadWidth=gravityArrowDiameter*Types.Defaults.ArrowHeadWidthFraction;
  parameter SI.Length gravityLineLength=max(0, gravityArrowLength - gravityHeadLength);
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowLine(
    shapeType="cylinder",
    length=gravityLineLength,
    width=gravityArrowDiameter,
    height=gravityArrowDiameter,
    lengthDirection=n,
    widthDirection={0,1,0},
    color=gravityArrowColor,
    r_shape=gravityArrowTail,
    specularCoefficient=0) if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravityArrowHead(
    shapeType="cone",
    length=gravityHeadLength,
    width=gravityHeadWidth,
    height=gravityHeadWidth,
    lengthDirection=n,
    widthDirection={0,1,0},
    color=gravityArrowColor,
    r_shape=gravityArrowTail + Modelica.Math.Vectors.normalize(
                                                n)*gravityLineLength,
    specularCoefficient=0) if enableAnimation and animateGravity and gravityType == GravityTypes.UniformGravity;

  // Point gravity visualization
  parameter Integer ndim_pointGravity=if enableAnimation and animateGravity
       and gravityType == GravityTypes.UniformGravity then 1 else 0;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape gravitySphere(
    shapeType="sphere",
    r_shape={-gravitySphereDiameter/2,0,0},
    lengthDirection={1,0,0},
    length=gravitySphereDiameter,
    width=gravitySphereDiameter,
    height=gravitySphereDiameter,
    color=gravitySphereColor,
    specularCoefficient=0) if enableAnimation and animateGravity and gravityType == GravityTypes.PointGravity;

  function gravityAcceleration = gravityAccelerationTypes (
      gravityType=gravityType,
      g=g*Modelica.Math.Vectors.normalize(
                                     n),
      mue=mue);

protected
  function gravityAccelerationTypes
    "Gravity field acceleration depending on field type and position"
    import Modelica.Mechanics.MultiBody.Types.GravityTypes;
    extends Modelica.Icons.Function;
    input SI.Position r[3]
      "Position vector from world frame to actual point, resolved in world frame";
    input GravityTypes gravityType "Type of gravity field";
    input SI.Acceleration g[3]
      "Constant gravity acceleration, resolved in world frame, if gravityType=1";
    input Real mue(unit="m3/s2")
      "Field constant of point gravity field, if gravityType=2";
    output SI.Acceleration gravity[3]
      "Gravity acceleration at point r, resolved in world frame";
  algorithm
    gravity := if gravityType == GravityTypes.UniformGravity then g else
               if gravityType == GravityTypes.PointGravity then
                  -(mue/(r*r))*(r/Modelica.Math.Vectors.length(
                                                r)) else
                    zeros(3);
    annotation(Inline=true);          
  end gravityAccelerationTypes;
equation
  Connections.root(frame_b.R);

  assert(Modelica.Math.Vectors.length(
                       n) > 1.e-10,
    "Parameter n of World object is wrong (lenght(n) > 0 required)");
  frame_b.r_0 = zeros(3);
  frame_b.R = Frames.nullRotation();

equation

end World;
