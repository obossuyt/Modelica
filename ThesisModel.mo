within ;
package ThesisModel



  model HeatTransferPipe

    Modelica.Thermal.HeatTransfer.Components.ThermalResistor Rfg(R=0.02)
      annotation (Placement(transformation(extent={{-22,8},{-2,28}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor Rgb(R=0.02)
      annotation (Placement(transformation(extent={{16,8},{36,28}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Cg(C=2e+8)
      "Heat capacity of the ground" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={6,-6})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TempBoundary
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={68,18})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(        redeclare package Medium
        = IDEAS.Media.Water, V=0.0001,
      nPorts=2,
      m_flow_nominal=pipeData.m_flow_nominal)
                             annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-60,-36})));
  Modelica.SIunits.Temperature T1;
  Modelica.SIunits.Temperature Tg;
  Modelica.SIunits.Temperature Tb;

    Modelica.Blocks.Sources.RealExpression TWall_val(y=283)
      "Average borehole wall temperature"
      annotation (Placement(transformation(extent={{48,-26},{70,-6}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a(
      redeclare final package Medium = Medium)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-94,2},{-74,22}}),
          iconTransformation(extent={{-122,-26},{-74,22}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b(
      redeclare final package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Constants.inf else 0))
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{118,-10},{98,10}}), iconTransformation(extent={{132,-28},
              {76,28}})));
    PipeData pipeData
      annotation (Placement(transformation(extent={{-86,72},{-66,92}})));
  equation
    connect(Rfg.port_b, Rgb.port_a) annotation (Line(
        points={{-2,18},{16,18}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Cg.port, Rgb.port_a) annotation (Line(
        points={{6,4},{6,18},{16,18}},
        color={191,0,0},
        smooth=Smooth.None));
  Rfg.port_a.T=T1;
  Rfg.port_b.T=Tg;
  Rgb.port_b.T=Tb;
  TempBoundary.T=10;

    connect(Rgb.port_b, TempBoundary.port) annotation (Line(
        points={{36,18},{58,18}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Rfg.port_a, vol.heatPort) annotation (Line(
        points={{-22,18},{-48,18},{-48,-36},{-50,-36}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(TempBoundary.T, TWall_val.y) annotation (Line(
        points={{80,18},{80,-16},{71.1,-16}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(vol.ports[1], port_a) annotation (Line(
        points={{-58,-26},{-74,-26},{-74,12},{-84,12}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(vol.ports[2], port_b) annotation (Line(
        points={{-62,-26},{-40,-26},{-40,0},{108,0}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_b, port_b) annotation (Line(
        points={{108,0},{108,0}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_a, port_a) annotation (Line(
        points={{-84,12},{-80,12},{-80,24},{-74,24},{-74,12},{-84,12}},
        color={0,127,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Rectangle(
            extent={{-100,62},{100,-54}},
            lineColor={0,0,0},
            fillColor={255,255,170},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-20,34},{28,-30}},
            lineColor={0,0,255},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-20,46},{28,22}},
            lineColor={0,0,255},
            fillColor={95,95,95},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-20,-22},{28,-40}},
            lineColor={0,0,255},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid)}));
  end HeatTransferPipe;

  model ExamplePipe

    package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

    HeatTransferPipe pipe(redeclare package Medium = Medium, pipeData=pipeData)
      annotation (Placement(transformation(extent={{-18,-30},{18,6}})));

    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
          Medium, m_flow_nominal=pipeData.m_flow_nominal)
      annotation (Placement(transformation(extent={{-16,30},{4,50}})));
    IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
      redeclare package Medium = Medium,
      m_flow_nominal=pipeData.m_flow_nominal,
      dp_nominal=pipeData.m_flow_nominal)
      annotation (Placement(transformation(extent={{36,30},{56,50}})));
    Modelica.Blocks.Sources.Step step(height=283)
      annotation (Placement(transformation(extent={{8,70},{28,90}})));
    PipeData pipeData
      annotation (Placement(transformation(extent={{-90,66},{-70,86}})));
  equation
    connect(fan.port_a, pipe.port_a) annotation (Line(
        points={{-16,40},{-28,40},{-28,-12.36},{-17.64,-12.36}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(fan.port_b, hea.port_a) annotation (Line(
        points={{4,40},{36,40}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipe.port_b, hea.port_b) annotation (Line(
        points={{18.72,-12},{68,-12},{68,40},{56,40}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(hea.TSet, step.y) annotation (Line(
        points={{34,46},{32,46},{32,80},{29,80}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics));
  end ExamplePipe;

  record PipeData
    "Record containing all the subrecords which describe all parameter values of the borefield"
    extends Modelica.Icons.Record;

    parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.01
      "Total nominal flow to the borefield";
    parameter Modelica.SIunits.Pressure dp_nominal=1000 "Total pressure drop";

  end PipeData;

  annotation (uses(
      IDEAS(version="0.2"),
      Buildings(version="2.1.0"),
      Modelica(version="3.2.1")));
end ThesisModel;
