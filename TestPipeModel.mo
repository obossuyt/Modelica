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
    Real SoC_teller;
    Real SoC_noemer;
    Real SoC;
    parameter Modelica.SIunits.Temperature Tref=pipeData.Tref;
    parameter Modelica.SIunits.Temperature Tmax=pipeData.Tmax;
    parameter Modelica.SIunits.Density rho=1000 "Density of the medium";
    parameter Modelica.SIunits.HeatCapacity C_water=4200
      "heat capacity of medium (now water)";

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
          origin={108,62}), iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={102,-58})));

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
    SoC_teller = Cg.C*(Cg.T - Tref) + C_water*(Cg.T - Tref);
    // ThermodynamicState=Medium.ThermodynamicState.T;
    // d = Medium.density(ThermodynamicState.T);
    // Referentietemperaturen van capaciteiten moeten verschillend zijn?
    // Maximum temperaturen van capaciteiten moeten verschillend zijn
    // De dichtheid, capaciteit en temperatuur van het watervolume: hoe uit Medium Package halen?

    SoC_noemer = Cg.C*(Tmax - Tref) + C_water*(Tmax - Tref);

    SoC = SoC_teller/SoC_noemer;

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Rectangle(
            extent={{-100,58},{100,-86}},
            lineColor={0,0,0},
            fillColor={255,255,170},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-20,-54},{28,-72}},
            lineColor={95,95,95},
            fillColor={215,215,215},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-20,2},{28,-62}},
            lineColor={95,95,95},
            fillColor={215,215,215},
            fillPattern=FillPattern.VerticalCylinder),
          Ellipse(
            extent={{-20,14},{28,-10}},
            lineColor={95,95,95},
            fillColor={95,95,95},
            fillPattern=FillPattern.VerticalCylinder),
          Rectangle(
            extent={{-88,32},{36,28}},
            lineColor={0,0,255},
            fillPattern=FillPattern.Solid,
            fillColor={0,0,255}),
          Polygon(
            points={{36,44},{36,18},{72,32},{36,44}},
            lineColor={0,0,255},
            smooth=Smooth.None,
            fillColor={0,0,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-58,92},{52,64}},
            lineColor={0,0,255},
            textString="%name")}));
  end PipeHeatTransfer;

  model Example01

    package Medium =
        IDEAS.Media.Specialized.Water.TemperatureDependentDensity;

    PipeHeatTransfer pipeHeatTransfer;
    PipeHeatTransfer pipe
      "Heat transfer in single pipe, flow from port_a to port_b"
      annotation (Placement(transformation(extent={{-16,-36},{4,-16}})));
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
      annotation (Placement(transformation(extent={{-76,30},{-56,50}})));

  equation
    connect(hea.TSet, step.y) annotation (Line(
        points={{42,10},{50,10},{50,44},{57.4,44}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pipe.T1, TWall_val.y) annotation (Line(
        points={{4.2,-31.8},{23.4,-31.8},{23.4,-42},{36.9,-42}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fan.port_a, hea.port_b) annotation (Line(
        points={{-36,16},{20,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipe.port_b, hea.port_a) annotation (Line(
        points={{4.4,-26},{68,-26},{68,16},{40,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipe.port_a, fan.port_b) annotation (Line(
        points={{-15.8,-26.2},{-68,-26.2},{-68,16},{-56,16}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(const.y, fan.m_flow_in) annotation (Line(
        points={{-8.7,-3},{-45.8,-3},{-45.8,4}},
        color={0,0,127},
        smooth=Smooth.None));
      annotation (Placement(transformation(extent={{-8,-26},{12,-6}})),
                Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics));
  end Example01;

  record PipeData

    parameter Modelica.SIunits.MassFlowRate m_flow_nominal=0.016
      "nominal mass flow rate";
    parameter Modelica.SIunits.Volume V=0.001 "volume";
    parameter Modelica.SIunits.Pressure dp_nominal=0.001
      "nominal pressure difference";
    parameter Modelica.SIunits.Temperature TempWall=283 "Wall Temperature";
    parameter Modelica.SIunits.Temperature Tref=283 "Reference Temperature";
    parameter Modelica.SIunits.Temperature Tmax=288 "Maximum Temperature";

  end PipeData;
  annotation (uses(Modelica(version="3.2.1"), IDEAS(version="0.2")));
end TestPipeModel;
