within ;
package TestPipeModel
  model PipeHeatTransfer
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor Rfg(R=0.02)
      annotation (Placement(transformation(extent={{-44,52},{-24,72}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor Rgb(R=0.02)
      annotation (Placement(transformation(extent={{6,52},{26,72}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Cg(C=2e+8)
      "Heat capacity of the ground" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-2,40})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TempBoundary
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={60,62})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(        redeclare package Medium
        = IDEAS.Media.Water,
      nPorts=2,
      m_flow_nominal=pipeData.m_flow,
      V=pipeData.V)          annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-68,-36})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a(
      redeclare final package Medium = Medium)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-114,-8},{-94,12}}),
          iconTransformation(extent={{-122,-26},{-74,22}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b(
      redeclare final package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Constants.inf else 0))
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{110,-10},{90,10}}), iconTransformation(extent={{132,-28},
              {76,28}})));
    Modelica.Blocks.Interfaces.RealInput T1 annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={108,62})));
    parameter Modelica.SIunits.Volume V=0.0001;
  equation
    connect(Rfg.port_b,Rgb. port_a) annotation (Line(
        points={{-24,62},{6,62}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Cg.port,Rgb. port_a) annotation (Line(
        points={{-2,50},{-2,62},{6,62}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Rgb.port_b,TempBoundary. port) annotation (Line(
        points={{26,62},{50,62}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Rfg.port_a,vol. heatPort) annotation (Line(
        points={{-44,62},{-56,62},{-56,-36},{-58,-36}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(vol.ports[1],port_a)  annotation (Line(
        points={{-66,-26},{-82,-26},{-82,2},{-104,2}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(vol.ports[2],port_b)  annotation (Line(
        points={{-70,-26},{-48,-26},{-48,0},{100,0}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_b,port_b)  annotation (Line(
        points={{100,0},{100,0}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(TempBoundary.T, T1) annotation (Line(
        points={{72,62},{108,62}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
              -100,-100},{100,100}}), graphics), Icon(graphics={
          Rectangle(
            extent={{-100,88},{100,-56}},
            lineColor={0,0,0},
            fillColor={255,255,170},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-20,-24},{28,-42}},
            lineColor={95,95,95},
            fillColor={215,215,215},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-20,32},{28,-32}},
            lineColor={95,95,95},
            fillColor={215,215,215},
            fillPattern=FillPattern.VerticalCylinder),
          Ellipse(
            extent={{-20,44},{28,20}},
            lineColor={95,95,95},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder)}));
  end PipeHeatTransfer;
  annotation (uses(Modelica(version="3.2.1"), IDEAS(version="0.2")));
  model Example01

    PipeHeatTransfer pipeHeatTransfer(V=pipeData.V)
      annotation (Placement(transformation(extent={{-8,-26},{12,-6}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
          Medium, m_flow_nominal=pipeData.m_flow)
      annotation (Placement(transformation(extent={{-34,6},{-14,26}})));
    IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
      redeclare package Medium = Medium,
      m_flow_nominal=pipeData.m_flow,
      dp_nominal=pipeData.dp_nominal)
      annotation (Placement(transformation(extent={{18,6},{38,26}})));
    Modelica.Blocks.Sources.Step step(height=283)
      annotation (Placement(transformation(extent={{-8,38},{4,50}})));
    Modelica.Blocks.Sources.RealExpression TWall_val(y=pipeData.TempWall)
      "Average borehole wall temperature"
      annotation (Placement(transformation(extent={{-11,-10},{11,10}},
          rotation=180,
          origin={51,-36})));
    Modelica.Blocks.Sources.Constant const
      annotation (Placement(transformation(extent={{-50,40},{-36,54}})));

    PipeData pipeData
      annotation (Placement(transformation(extent={{-90,72},{-70,92}})));

  equation
    connect(fan.port_b,hea. port_a) annotation (Line(
        points={{-14,16},{18,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(hea.TSet,step. y) annotation (Line(
        points={{16,22},{14,22},{14,44},{4.6,44}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pipeHeatTransfer.port_a, fan.port_a) annotation (Line(
        points={{-7.8,-16.2},{-7.8,-16.1},{-34,-16.1},{-34,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipeHeatTransfer.port_b, hea.port_b) annotation (Line(
        points={{12.4,-16},{62,-16},{62,16},{38,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipeHeatTransfer.T1, TWall_val.y) annotation (Line(
        points={{12.8,-9.8},{12.8,-8.9},{38.9,-8.9},{38.9,-36}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const.y, fan.m_flow_in) annotation (Line(
        points={{-35.3,47},{-24,47},{-24,28},{-24.2,28}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics));
  end Example01;

  record PipeData

    parameter Modelica.SIunits.MassFlowRate m_flow=0.016
      "nominal mass flow rate";
    parameter Modelica.SIunits.Volume V=0.001 "volume";
    parameter Modelica.SIunits.Pressure dp_nominal=0.001
      "nominal pressure difference";
    parameter Modelica.SIunits.Temperature TempWall=283 "Wall Temperature";
  end PipeData;
end TestPipeModel;
