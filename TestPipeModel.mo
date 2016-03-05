within ;
package TestPipeModel
  model PipeHeatTransfer

    Modelica.Thermal.HeatTransfer.Components.ThermalResistor Rfg(R=0.02)
      annotation (Placement(transformation(extent={{-26,52},{-6,72}})));
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

    replaceable package Medium =
        IDEAS.Media.Specialized.Water.TemperatureDependentDensity;
    Real SoC_teller;
    Real SoC_noemer;
    Real SoC;
    parameter Modelica.SIunits.Temperature Tref=pipeData.Tref;
    parameter Modelica.SIunits.Temperature Tmax=pipeData.Tmax;

    Medium.ThermodynamicState sta_default=Medium.setState_pTX(
        p=port_a.p,
        T=tempSensor.T,
        X=port_a.Xi_outflow);

    Modelica.SIunits.HeatCapacity Cp_fluid=Medium.specificHeatCapacityCp(state=
        sta_default) "Specific heat capacity of medium";

    Modelica.SIunits.Density rho=Medium.density(state=sta_default)
      "Density of medium";

    //  parameter Modelica.SIunits.HeatCapacity Cp_fluid=4200;
    // parameter Modelica.SIunits.Density rho=1000;

    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      nPorts=2,
      m_flow_nominal=pipeData.m_flow_nominal,
      V=pipeData.V) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-52,-64})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package Medium
        = Medium)
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-112,-8},{-92,12}}),
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

    IDEAS.Fluid.FixedResistances.FixedResistanceDpM res(
      redeclare package Medium = Medium,
      m_flow_nominal=pipeData.m_flow_nominal,
      dp_nominal=pipeData.dp_nominal)
      annotation (Placement(transformation(extent={{-92,-10},{-74,12}})));

    parameter Modelica.SIunits.Volume V=pipeData.V;

    Modelica.Fluid.Sensors.Temperature tempSensor(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-50,-36},{-30,-16}})));

  equation
    connect(Rfg.port_b, Rgb.port_a) annotation (Line(
        points={{-6,62},{6,62}},
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
        points={{-26,62},{-30,62},{-30,-64},{-42,-64}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(vol.ports[1], port_b) annotation (Line(
        points={{-50,-54},{34,-54},{34,0},{100,0}},
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

    // SoC_teller = Cg.C*(Cg.T - Tref) + Cp_fluid*rho*V*(Cg.T - Tref);
    SoC_teller = Cg.C*(Cg.T - Tref) + Cp_fluid*(Cg.T - Tref);
    // Cp_fluid [J/K]
    // ThermodynamicState=Medium.ThermodynamicState.T;
    // d = Medium.density(ThermodynamicState.T);
    // Referentietemperaturen van capaciteiten moeten verschillend zijn?
    // Maximum temperaturen van capaciteiten moeten verschillend zijn
    // De dichtheid, capaciteit en temperatuur van het watervolume: hoe uit Medium Package halen?

    SoC_noemer = Cg.C*(Tmax - Tref) + Cp_fluid*(Tmax - Tref);

    SoC = SoC_teller/SoC_noemer;

    connect(res.port_a, port_a) annotation (Line(
        points={{-92,1},{-96,1},{-96,2},{-102,2}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_a, port_a) annotation (Line(
        points={{-102,2},{-102,2}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(vol.ports[2], res.port_b) annotation (Line(
        points={{-54,-54},{-62,-54},{-62,1},{-74,1}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(tempSensor.port, res.port_b) annotation (Line(
        points={{-40,-36},{-62,-36},{-62,1},{-74,1}},
        color={0,127,255},
        smooth=Smooth.None));
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

   // PipeHeatTransfer pipeHeatTransfer;
    PipeHeatTransfer pipe
      "Heat transfer in single pipe, flow from port_a to port_b"
      annotation (Placement(transformation(extent={{-16,-36},{4,-16}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
        redeclare package Medium = Medium, m_flow_nominal=pipeData.m_flow_nominal,
      motorCooledByFluid=false)
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
    Modelica.Blocks.Sources.Step step(
      height=10,
      startTime=500,
      offset=274)
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=180,
          origin={64,42})));
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
      annotation (Placement(transformation(extent={{-88,74},{-68,94}})));

    Modelica.Fluid.Sources.Boundary_pT boundary(          redeclare package
        Medium = Medium, nPorts=1)
      annotation (Placement(transformation(extent={{-88,32},{-68,52}})));
    //IDEAS.Media.Specialized.Water.TemperatureDependentDensity.ThermodynamicState
      //thermodynamicState
     // annotation (Placement(transformation(extent={{-26,70},{-6,90}})));
  equation
    connect(hea.TSet, step.y) annotation (Line(
        points={{42,10},{50,10},{50,42},{57.4,42}},
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
    connect(fan.port_b, boundary.ports[1]) annotation (Line(
        points={{-56,16},{-60,16},{-60,42},{-68,42}},
        color={0,127,255},
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
