within ;
package StateOfCharge

replaceable package Medium =
      IDEAS.Media.Specialized.Water.TemperatureDependentDensity;

public
  model MultipleBoreHolesUTube
    "Borefield model using single U-tube borehole heat exchanger configuration."

    // Medium in borefield
    extends
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Interfaces.partial_multipleBoreHoles;
        replaceable
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.SingleBoreHolesInSerie
      borHolSer(
      redeclare final package Medium = Medium,
      final soi=bfData.soi,
      final fil=bfData.fil,
      final gen=bfData.gen,
      final energyDynamics=energyDynamics,
      final massDynamics=massDynamics,
      final T_start=T_start,
      final dynFil=dynFil,
      final mSenFac=mSenFac,
      final use_TWall=true,
      final m_flow_nominal=m_flow_nominal/bfData.gen.nbBh*bfData.gen.nbSer,
      final dp_nominal=dp_nominal) constrainedby
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.Interface.PartialSingleBoreholeSerie(
      redeclare package Medium = Medium,
      soi=bfData.soi,
      fil=bfData.fil,
      gen=bfData.gen,
      energyDynamics=energyDynamics,
      massDynamics=massDynamics,
      T_start=T_start,
      dynFil=dynFil,
      mSenFac=mSenFac,
      use_TWall=true,
      m_flow_nominal=m_flow_nominal/bfData.gen.nbBh*bfData.gen.nbSer,
      dp_nominal=dp_nominal) "NbSer boreholes in series" annotation (Placement(
          transformation(
          extent={{12,13},{-12,-13}},
          rotation=180,
          origin={-1,0})));

  equation
    connect(massFlowRateMultiplier.port_b, borHolSer.port_a)
      annotation (Line(points={{-60,0},{-13,0}},         color={0,127,255}));
    connect(borHolSer.port_b, massFlowRateMultiplier1.port_a)
      annotation (Line(points={{11,0},{60,0}},        color={0,127,255}));
    connect(TWall_val.y, borHolSer.TWall)
      annotation (Line(points={{-18.9,40},{-1,40},{-1,14.3}}, color={0,0,127}));
    annotation (
      experiment(StopTime=70000, __Dymola_NumberOfIntervals=50),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Rectangle(
            extent={{-100,60},{100,-66}},
            lineColor={0,0,0},
            fillColor={234,210,210},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-88,-6},{-32,-62}},
            lineColor={0,0,0},
            fillColor={223,188,190},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-82,-12},{-38,-56}},
            lineColor={0,0,0},
            fillColor={0,0,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-88,54},{-32,-2}},
            lineColor={0,0,0},
            fillColor={223,188,190},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-82,48},{-38,4}},
            lineColor={0,0,0},
            fillColor={0,0,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-26,54},{30,-2}},
            lineColor={0,0,0},
            fillColor={223,188,190},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-20,48},{24,4}},
            lineColor={0,0,0},
            fillColor={0,0,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-28,-6},{28,-62}},
            lineColor={0,0,0},
            fillColor={223,188,190},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{-22,-12},{22,-56}},
            lineColor={0,0,0},
            fillColor={0,0,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{36,56},{92,0}},
            lineColor={0,0,0},
            fillColor={223,188,190},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{42,50},{86,6}},
            lineColor={0,0,0},
            fillColor={0,0,255},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{38,-4},{94,-60}},
            lineColor={0,0,0},
            fillColor={223,188,190},
            fillPattern=FillPattern.Forward),
          Ellipse(
            extent={{44,-10},{88,-54}},
            lineColor={0,0,0},
            fillColor={0,0,255},
            fillPattern=FillPattern.Forward)}),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}})),Documentation(info="<html>
  <p>The proposed model is a so-called hybrid step-response
model (HSRM). This type of model uses the
borefield’s temperature response to a step load input.
An arbitrary load can always be approximated by a superposition
of step loads. The borefield’s response to
the load is then calculated by superposition of the step responses
using the linearity property of the heat diffusion
equation. The most famous example of HSRM
for borefields is probably the <i>g-function</i> of Eskilson
(1987). The major challenge of this approach is to obtain a
HSRM which is valid for both minute-based and year-based
simulations. To tackle this problem, a HSRM
has been implemented. A long-term response model
is implemented in order to take into account
the interactions between the boreholes and the
temperature evolution of the surrounding ground. A
short-term response model is implemented to
describe the transient heat flux in the borehole heat exchanger to the surrounding
ground. The step-response of each model is then calculated and merged into one
in order to achieve both short- and long-term
accuracy. Finally an aggregation method is implemented to speed up the calculation.
However, the aggregation method calculates the temperature for discrete time step. In order to avoid
abrut temperature changes, the aggregation method is used to calculate the average borehole wall
temperature instead of the average fluid temperature. The calculated borehole wall temperature is then
connected to the dynamic model of the borehole heat exchanger.</p>
<p>More detailed documentation can be found in 
<a href=\"modelica://IDEAS/Resources/Images/Fluid/HeatExchangers/BroundHeatExchangers/Borefield/UsersGuide/2014-10thModelicaConference-Picard.pdf\">Picard (2014)</a>.
and in 
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.UsersGuide\">IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.UsersGuide</a>.
</p>
<p>
A verification of this model can be found in 
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Validation.TrtValidation\">TrtValidation</a>
.
</p>
</html>",   revisions="<html>
<ul>
<li>
July 2014, by Damien Picard:<br>
First implementation.
</li>
</ul>
</html>"));
  end MultipleBoreHolesUTube;

  package BfData
    extends
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Data.Records.BorefieldData;
  end BfData;

  model IntHEX_SoC
    extends
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.InternalHEXUTube;

    Medium.ThermodynamicState sta_default=Medium.setState_pTX(
        Medium.p_default,
        Medium.T_default,
        Medium.X_default) "Thermodynamic state";

    parameter Modelica.Media.Interfaces.Types.Density rhoMed=Medium.density(state=
        sta_default);

    parameter Modelica.Media.Interfaces.Types.Temperature tempMed=
        Medium.temperature(state=sta_default);

    Modelica.SIunits.HeatCapacity CMed1=cpMed*rhoMed*vol1.V
      "heat capacity of volume 1";

    Modelica.SIunits.HeatCapacity CMed2=cpMed*rhoMed*vol2.V
      "heat capacity of volume 2";

    Modelica.SIunits.HeatCapacity Cap[:]={capFil1.C,capFil2.C,CMed1,CMed2};

    Modelica.SIunits.Temperature TCap[:]={capFil1.T,capFil2.T,tempMed,tempMed};
    Modelica.SIunits.Temperature TMax[:]={283,283,283,283};

    parameter Modelica.SIunits.Temperature Tref=274;

    Modelica.SIunits.Energy Num[:];
    Modelica.SIunits.Energy Denum[:];
    Modelica.SIunits.Energy HEXSoC_Num;
    Modelica.SIunits.Energy HEXSoC_Denum;

    Modelica.Blocks.Interfaces.RealOutput intHEXSoC_Num annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={104,-92})));
    Modelica.Blocks.Interfaces.RealOutput intHEXSoC_Denum annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={104,-116})));
  algorithm
    for i in 1:size(Cap, 1) loop
      Num[i] := Cap[i]*(TCap[i] - Tref);
      Denum[i] := Cap[i]*(TMax[i] - Tref);
    end for;

  algorithm
    for i in 1:size(Cap, 1) loop
      HEXSoC_Num := HEXSoC_Num + Num[i];
      HEXSoC_Denum := HEXSoC_Denum + Denum[i];
    end for;

  intHEXSoC_Num:=HEXSoC_Num;
  intHEXSoC_Num:=HEXSoC_Denum;

  equation
    connect(intHEXSoC_Num,intHEXSoC_Num)  annotation (Line(
        points={{104,-92},{104,-92}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -120},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-120},{100,100}}),
          graphics={Line(
            points={{94,-92},{20,-92},{20,-80}},
            color={0,0,255},
            smooth=Smooth.None), Line(
            points={{94,-116},{20,-116},{20,-86}},
            color={0,0,255},
            smooth=Smooth.None)}));
  end IntHEX_SoC;

  model SoilLay_SoC
    extends
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer;

    Modelica.SIunits.Temperature TMax[:]={283,283,283};

    parameter Modelica.SIunits.Temperature Tref=274;

    Modelica.SIunits.Energy Num[:];
    Modelica.SIunits.Energy Denum[:];
    Modelica.SIunits.Energy SoilSoC_Num;
    Modelica.SIunits.Energy SoilSoC_Denum;

    Modelica.Blocks.Interfaces.RealOutput soilLaySoC_Num
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput soilLaySoC_Denum
      annotation (Placement(transformation(extent={{96,26},{116,46}})));
  algorithm
    for i in 1:size(Cap, 1) loop
      Num[i] := C[i]*(T[i] - Tref);
      Denum[i] := C[i]*(TMax[i] - Tref);
    end for;

  algorithm
    for i in 1:size(Cap, 1) loop
      SoilSoC_Num := SoilSoC_Num + Num[i];
      SoilSoC_Denum := SoilSoC_Denum + Denum[i];
    end for;

  soilLaySoC_Num:=SoilSoC_Num;
  soilLaySoC_Denum:=SoilSoC_Denum;

  equation
    connect(soilLaySoC_Num,soilLaySoC_Num)  annotation (Line(
        points={{106,60},{106,60}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Line(
            points={{96,60},{60,60}},
            color={0,0,255},
            smooth=Smooth.None), Line(
            points={{96,36},{66,36}},
            color={0,0,255},
            smooth=Smooth.None)}));
  end SoilLay_SoC;

  model Example01
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
        redeclare package Medium = Medium, m_flow_nominal=pipeData.m_flow_nominal,
      motorCooledByFluid=false)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-36,26})));
    IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
      redeclare package Medium = Medium,
      m_flow_nominal=pipeData.m_flow_nominal,
      dp_nominal=pipeData.dp_nominal)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={40,26})));
    Modelica.Blocks.Sources.Step step(
      height=10,
      startTime=500,
      offset=274)
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=180,
          origin={74,52})));
    Modelica.Blocks.Sources.Constant const(k=pipeData.m_flow_nominal)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=180,
          origin={9,7})));
    Modelica.Fluid.Sources.Boundary_pT boundary(          redeclare package
        Medium = Medium, nPorts=2)
      annotation (Placement(transformation(extent={{-78,42},{-58,62}})));
    MultipleBoreHolesUTube_SoC multipleBoreHolesUTube_SoC
      annotation (Placement(transformation(extent={{-14,-38},{6,-18}})));
  equation
    connect(hea.TSet,step. y) annotation (Line(
        points={{52,20},{60,20},{60,52},{67.4,52}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fan.port_a,hea. port_b) annotation (Line(
        points={{-26,26},{30,26}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(const.y,fan. m_flow_in) annotation (Line(
        points={{1.3,7},{-35.8,7},{-35.8,14}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fan.port_b,boundary. ports[1]) annotation (Line(
        points={{-46,26},{-50,26},{-50,54},{-58,54}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(hea.port_a, multipleBoreHolesUTube_SoC.port_b) annotation (Line(
        points={{50,26},{66,26},{66,-28},{6,-28}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(multipleBoreHolesUTube_SoC.port_a, fan.port_b) annotation (Line(
        points={{-14,-28},{-72,-28},{-72,26},{-46,26}},
        color={0,127,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
              -100,-100},{100,100}}), graphics));
  end Example01;

  annotation (uses(Modelica(version="3.2.1"), IDEAS(version="0.2")));
end StateOfCharge;
