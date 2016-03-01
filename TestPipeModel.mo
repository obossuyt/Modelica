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

    package Medium = IDEAS.Media.Specialized.Water.TemperatureDependentDensity;

    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      nPorts=2,
      m_flow_nominal=pipeData.m_flow_nominal,
      V=pipeData.V) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-70,-36})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package Medium
        = Medium)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-114,-8},{-94,12}}),
          iconTransformation(extent={{-122,-26},{-74,22}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare final package Medium
        = Medium)
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{110,-10},{90,10}}),
          iconTransformation(extent={{132,-28},{76,28}})));
    Modelica.Blocks.Interfaces.RealInput T1 annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={108,62})));

    PipeData pipeData
      annotation (Placement(transformation(extent={{-92,64},{-72,84}})));
  equation
    connect(Rfg.port_b, Rgb.port_a) annotation (Line(
        points={{-24,62},{6,62}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Cg.port, Rgb.port_a) annotation (Line(
        points={{-2,50},{-2,62},{6,62}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Rgb.port_b, TempBoundary.port) annotation (Line(
        points={{26,62},{50,62}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(Rfg.port_a, vol.heatPort) annotation (Line(
        points={{-44,62},{-56,62},{-56,-36},{-60,-36}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(vol.ports[1], port_a) annotation (Line(
        points={{-68,-26},{-82,-26},{-82,2},{-104,2}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(vol.ports[2], port_b) annotation (Line(
        points={{-72,-26},{-48,-26},{-48,0},{100,0}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_b, port_b) annotation (Line(
        points={{100,0},{100,0}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(TempBoundary.T, T1) annotation (Line(
        points={{72,62},{108,62}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                                                 graphics={
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
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-88,68},{36,64}},
            lineColor={0,0,255},
            fillPattern=FillPattern.Solid,
            fillColor={0,0,255}),
          Polygon(
            points={{36,78},{36,52},{72,66},{36,78}},
            lineColor={0,0,255},
            smooth=Smooth.None,
            fillColor={0,0,255},
            fillPattern=FillPattern.Solid)}));
  end PipeHeatTransfer;

  model Example01

    package Medium =
        IDEAS.Media.Specialized.Water.TemperatureDependentDensity;

    PipeHeatTransfer pipeHeatTransfer;
    PipeHeatTransfer pipeHeatTransfer1
      "Heat transfer in single pipe, flow from port_a to port_b"
      annotation (Placement(transformation(extent={{-12,-38},{8,-18}})));
      annotation (Placement(transformation(extent={{-8,-26},{12,-6}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
        redeclare package Medium = Medium, m_flow_nominal=pipeData.m_flow_nominal)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-46,16})));
    IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
      redeclare package Medium = Medium,
      m_flow_nominal=pipeData.m_flow_nominal,
      dp_nominal=pipeData.dp_nominal)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={30,16})));
    Modelica.Blocks.Sources.Step step(height=283)
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=180,
          origin={64,44})));
    Modelica.Blocks.Sources.RealExpression TWall_val(y=pipeData.TempWall)
      "Average borehole wall temperature" annotation (Placement(transformation(
          extent={{-11,-10},{11,10}},
          rotation=180,
          origin={49,-42})));
    Modelica.Blocks.Sources.Constant const(k=pipeData.m_flow_nominal)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=180,
          origin={-1,-3})));

    PipeData pipeData
      annotation (Placement(transformation(extent={{-90,72},{-70,92}})));

  equation
    connect(hea.TSet, step.y) annotation (Line(
        points={{42,10},{50,10},{50,44},{57.4,44}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pipeHeatTransfer1.T1, TWall_val.y) annotation (Line(
        points={{8.8,-21.8},{23.4,-21.8},{23.4,-42},{36.9,-42}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fan.port_a, hea.port_b) annotation (Line(
        points={{-36,16},{20,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipeHeatTransfer1.port_b, hea.port_a) annotation (Line(
        points={{8.4,-28},{68,-28},{68,16},{40,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipeHeatTransfer1.port_a, fan.port_b) annotation (Line(
        points={{-11.8,-28.2},{-68,-28.2},{-68,16},{-56,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(const.y, fan.m_flow_in) annotation (Line(
        points={{-8.7,-3},{-45.8,-3},{-45.8,4}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics));
  end Example01;

  record PipeData

    parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.016
      "nominal mass flow rate";
    parameter Modelica.SIunits.Volume V=0.001 "volume";
    parameter Modelica.SIunits.Pressure dp_nominal=0.001
      "nominal pressure difference";
    parameter Modelica.SIunits.Temperature TempWall=283 "Wall Temperature";

  end PipeData;
  annotation (uses(Modelica(version="3.2.1"), IDEAS(version="0.2")));
end TestPipeModel;
