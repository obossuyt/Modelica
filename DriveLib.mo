within ;
package DriveLib
  model Motor "A basic model of an electrical motor"
    Modelica.Electrical.Analog.Basic.Resistor Ra(R=0.5)
      annotation (Placement(transformation(extent={{-42,28},{-22,48}})));
    Modelica.Electrical.Analog.Basic.Inductor La(L=0.05, i(fixed=true))
      annotation (Placement(transformation(extent={{-8,28},{12,48}})));
    Modelica.Electrical.Analog.Basic.Ground G
      annotation (Placement(transformation(extent={{-18,-30},{2,-10}})));
    Modelica.Electrical.Analog.Basic.EMF emf(k=1)
      annotation (Placement(transformation(extent={{28,2},{48,22}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage Vs annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-70,14})));
    Modelica.Mechanics.Rotational.Components.Inertia Jm(
      J=0.001,
      phi(fixed=true),
      w(fixed=true))
      annotation (Placement(transformation(extent={{66,2},{86,22}})));
    Modelica.Blocks.Interfaces.RealInput v1
      "Voltage between pin p and n (= p.v - n.v) as input signal" annotation (
        Placement(transformation(extent={{-126,-6},{-86,34}}),
          iconTransformation(extent={{-126,-6},{-86,34}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
      "Right flange of shaft" annotation (Placement(transformation(extent={{90,
              2},{110,22}}), iconTransformation(extent={{90,2},{108,20}})));
  equation
    connect(Ra.n, La.p) annotation (Line(
        points={{-22,38},{-8,38}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(emf.flange, Jm.flange_a) annotation (Line(
        points={{48,12},{66,12}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(La.n, emf.p) annotation (Line(
        points={{12,38},{38,38},{38,22}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(emf.n, G.p) annotation (Line(
        points={{38,2},{38,-10},{-8,-10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(Ra.p, Vs.p) annotation (Line(
        points={{-42,38},{-70,38},{-70,24}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(G.p, Vs.n) annotation (Line(
        points={{-8,-10},{-70,-10},{-70,4}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(Vs.v, v1) annotation (Line(
        points={{-77,14},{-106,14}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(Jm.flange_b, flange_b1) annotation (Line(
        points={{86,12},{100,12}},
        color={0,0,0},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={
          Rectangle(
            extent={{-56,44},{60,-36}},
            lineColor={0,0,0},
            fillColor={127,0,0},
            fillPattern=FillPattern.HorizontalCylinder),
          Text(
            extent={{-42,80},{42,60}},
            lineColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={127,0,0},
            textString="%name"),
          Polygon(
            points={{-20,-12},{16,-12},{32,-58},{50,-58},{50,-74},{-58,-74},{
                -58,-58},{-38,-58},{-20,-12}},
            lineColor={0,0,0},
            fillPattern=FillPattern.HorizontalCylinder,
            smooth=Smooth.None,
            fillColor={0,0,0}),
          Line(
            points={{-88,14},{-56,14}},
            color={0,0,0},
            smooth=Smooth.None),
          Rectangle(
            extent={{60,20},{98,2}},
            lineColor={0,0,0},
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={135,135,135})}),
      Documentation(info="<html>
<p>This is a simple motor model.</p>
<p><br>This documentation can certainly be extended using e.g. links and images of this editor. An image of the topbar above</p>
</html>"));
  end Motor;
  annotation (uses(Modelica(version="3.2.1")));
  model TestMotor
    Motor motor annotation (Placement(transformation(extent={{-8,6},{12,26}})));
    Modelica.Blocks.Sources.Step step
      annotation (Placement(transformation(extent={{-56,8},{-36,28}})));
  equation
    connect(motor.v1, step.y) annotation (Line(
        points={{-8.6,17.4},{-22,17.4},{-22,18},{-35,18}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(graphics));
  end TestMotor;

  model MotorDrive

    Motor motor annotation (Placement(transformation(extent={{-4,-10},{20,12}})));
    Modelica.Blocks.Math.Feedback positionerror
      annotation (Placement(transformation(extent={{-92,-8},{-72,12}})));
    Modelica.Blocks.Continuous.PID controller
      annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
    Modelica.Mechanics.Rotational.Components.IdealGear gearbox(ratio=3)
      annotation (Placement(transformation(extent={{30,-8},{50,12}})));
    Modelica.Mechanics.Rotational.Components.Inertia load(J=0.5*m*r^2)
      annotation (Placement(transformation(extent={{64,-8},{84,12}})));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor phiload annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={86,-30})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=100, uMin=-100)
      annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
    parameter Modelica.SIunits.Radius r=0.5 "Radius of load";
    parameter Modelica.SIunits.Mass m=80 "mass of load";
  equation

    connect(motor.flange_b1, gearbox.flange_a) annotation (Line(
        points={{19.88,2.21},{22,2.21},{22,2},{30,2}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(phiload.flange, load.flange_b) annotation (Line(
        points={{86,-20},{86,2},{84,2}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(load.flange_a, gearbox.flange_b) annotation (Line(
        points={{64,2},{50,2}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(phiload.phi, positionerror.u2) annotation (Line(
        points={{86,-41},{-82,-41},{-82,-6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(controller.y, limiter.u) annotation (Line(
        points={{-41,0},{-34,0}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(motor.v1, limiter.y) annotation (Line(
        points={{-4.72,2.54},{-4,2.54},{-4,0},{-11,0}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(controller.u, positionerror.y) annotation (Line(
        points={{-64,0},{-70,0},{-70,2},{-73,2}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics));
  end MotorDrive;

  model MotorDriveTest
    extends MotorDrive(controller(
        Td=0.001,
        Ti=1000.0,
        k=2.0,
        D(x(fixed=true))));
    Modelica.Blocks.Sources.Step step
      annotation (Placement(transformation(extent={{-100,-8},{-80,12}})));
  equation
    connect(positionerror.u1, step.y) annotation (Line(
        points={{-90,2},{-79,2}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(graphics));
  end MotorDriveTest;
end DriveLib;
