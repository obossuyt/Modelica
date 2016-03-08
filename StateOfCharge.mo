within ;
package StateOfCharge

replaceable package Medium =
      IDEAS.Media.Specialized.Water.TemperatureDependentDensity;


  package BfData
    extends
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Data.Records.BorefieldData;
  end BfData;


  package BaseClasses
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
      for i in 1:size(C, 1) loop
        Num[i] := C[i]*(T[i] - Tref);
        Denum[i] := C[i]*(TMax[i] - Tref);
      end for;

    algorithm
      for i in 1:size(C, 1) loop
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
            Rectangle(
              extent={{86,80},{124,16}},
              lineColor={0,0,255},
              fillColor={255,255,170},
              fillPattern=FillPattern.Solid),
              Line(
              points={{96,60},{60,60}},
              color={0,0,255},
              smooth=Smooth.None), Line(
              points={{96,36},{66,36}},
              color={0,0,255},
              smooth=Smooth.None)}));
    end SoilLay_SoC;

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
            graphics={
            Rectangle(
              extent={{84,-72},{122,-136}},
              lineColor={0,0,255},
              fillColor={255,255,170},
              fillPattern=FillPattern.Solid),
                      Line(
              points={{94,-92},{20,-92},{20,-80}},
              color={0,0,255},
              smooth=Smooth.None), Line(
              points={{94,-116},{20,-116},{20,-86}},
              color={0,0,255},
              smooth=Smooth.None)}));
    end IntHEX_SoC;

    model BoreHoleSegmentFourPort_SoC "Vertical segment of a borehole"
      extends PartialBoreHoleSegment_SoC;

      extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
        redeclare final package Medium1 = Medium,
        redeclare final package Medium2 = Medium,
        final m1_flow_small=gen.m_flow_small,
        final m2_flow_small=gen.m_flow_small,
        final allowFlowReversal1=gen.allowFlowReversal,
        final allowFlowReversal2=gen.allowFlowReversal);

      //Real SoC=(SoilLaySoC_Num + IntHEXSoC_Num)/(SoilLaySoC_Denum + IntHEXSoC_Denum);

      Modelica.SIunits.Energy SoC_Num=soilLay_SoC.soilLaySoC_Num + intHEX_SoC.intHEXSoC_Num;
      Modelica.SIunits.Energy SoC_Denum=soilLay_SoC.soilLaySoC_Denum + intHEX_SoC.intHEXSoC_Denum;

      Real StateOfCharge=SoC_Num/SoC_Denum;

      IntHEX_SoC intHEX_SoC
        annotation (Placement(transformation(extent={{-70,-12},{-50,10}})));
      Modelica.Blocks.Interfaces.RealOutput SoC annotation (Placement(
            transformation(extent={{-6,-104},{14,-84}}),
                                                       iconTransformation(extent={{-12,-12},
                {12,12}},
            rotation=270,
            origin={0,-104})));
    equation
      if not use_TWall then
      else
      end if;
      connect(soilLay_SoC.port_a, intHEX_SoC.port) annotation (Line(
          points={{-60,26},{-60,10}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(intHEX_SoC.port_b1, port_b1) annotation (Line(
          points={{-50,6},{26,6},{26,60},{100,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(intHEX_SoC.port_a2, port_a2) annotation (Line(
          points={{-50,-6},{26,-6},{26,-60},{100,-60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(intHEX_SoC.port_b2, port_b2) annotation (Line(
          points={{-70,-6},{-86,-6},{-86,-60},{-100,-60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(intHEX_SoC.port_a1, port_a1) annotation (Line(
          points={{-70,6},{-84,6},{-84,60},{-100,60}},
          color={0,127,255},
          smooth=Smooth.None));
      SoC =  StateOfCharge;

      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}),
                        graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}),
            graphics={
            Rectangle(
              extent={{-38,-86},{36,-112}},
              lineColor={0,0,255},
              fillColor={255,255,170},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-72,80},{68,-80}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{88,54},{-88,64}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{88,-64},{-88,-54}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-72,80},{68,68}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Backward),
            Rectangle(
              extent={{-72,-68},{68,-80}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Backward),
            Line(
              points={{0,-84},{0,-92}},
              color={0,0,255},
              smooth=Smooth.None)}),
        Documentation(info="<html>
<p>
Horizontal layer that is used to model a U-tube borehole heat exchanger. 
This model combines three models, each simulating a different aspect 
of a borehole heat exchanger. 
</p>
<p>
The instance <code>intHEX</code> computes the heat transfer in the pipes and the filling material. 
This computation is done using the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.SingleUTubeInternalHEX\">
IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.SingleUTubeInternalHEX</a>.
</p>
<p>
The instance <code>soiLay</code> computes transient and steady state heat transfer in the soil using a vertical cylinder.
The computation is done using the model <a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer\">
IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer</a>.
</p>
<p>
The model <code>TBouCon</code> is a constant temperature equal to the initial ground temperature.</a>.
</p>
</html>",     revisions="<html>
<ul>
<li>
July 2014, by Damien Picard:<br>
First implementation.
</li>
</ul>
</html>"));
    end BoreHoleSegmentFourPort_SoC;

    partial model PartialBoreHoleSegment_SoC
      extends
        IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.Interface.PartialBoreHoleElement;
      extends IDEAS.Fluid.Interfaces.TwoPortFlowResistanceParameters(
          computeFlowResistance=false, linearizeFlowResistance=false);
      extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations;
      extends
        IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.Interface.PartialTWall;
      parameter Modelica.SIunits.Temperature TExt_start=T_start
        "Initial far field temperature"
        annotation (Dialog(tab="Boundary conditions",group="T_start: ground"));
      parameter Modelica.SIunits.Temperature TFil_start=T_start
        "Initial far field temperature"
        annotation (Dialog(tab="Boundary conditions",group="T_start: ground"));

      Modelica.Blocks.Sources.RealExpression realExpression(final y=T_start) if not use_TWall
        annotation (Placement(transformation(extent={{-30,80},{-50,100}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TBouCon
        "Thermal boundary condition for the far-field"
        annotation (Placement(transformation(extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-60,70})));

      SoilLay_SoC soilLay_SoC annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,36})));
    equation
      if not use_TWall then
        connect(realExpression.y,TBouCon. T) annotation (Line(
          points={{-51,90},{-60,90},{-60,82}},
          color={0,0,127},
          smooth=Smooth.None));
      else
        connect(TBouCon.T, TWall) annotation (Line(points={{-60,82},{-60,82},{0,
                82},{0,102}},
                   color={0,0,127}));
      end if;

      connect(TBouCon.port, soilLay_SoC.port_b) annotation (Line(
          points={{-60,60},{-60,46}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics));
    end PartialBoreHoleSegment_SoC;

    model SingleBoreHoleUTube_SoC "Single U-tube borehole heat exchanger"

      extends
        IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.Interface.PartialSingleBoreHole(
         T_start=gen.T_start);

      StateOfCharge.BaseClasses.BoreHoleSegmentFourPort_SoC
        borHolSeg[gen.nVer](
        redeclare each final package Medium = Medium,
        each final soi=soi,
        each final fil=fil,
        each final gen=gen,
        each final TExt_start=T_start,
        each final TFil_start=T_start,
        each final show_T=show_T,
        each final computeFlowResistance=computeFlowResistance,
        each final from_dp=from_dp,
        each final linearizeFlowResistance=linearizeFlowResistance,
        each final deltaM=deltaM,
        each final energyDynamics=energyDynamics,
        each final massDynamics=massDynamics,
        each final p_start=p_start,
        each final T_start=T_start,
        each final X_start=X_start,
        each final C_start=C_start,
        each final C_nominal=C_nominal,
        each final dynFil=dynFil,
        each final mSenFac=mSenFac,
        each final use_TWall=use_TWall,
        final dp_nominal={if i == 1 then dp_nominal else 0 for i in 1:gen.nVer},
        each final m1_flow_nominal=m_flow_nominal,
        each final m2_flow_nominal=m_flow_nominal)
        "Discretized borehole segments"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

    equation
      TWallAve = sum(borHolSeg[:].intHEX_SoC.port.T)/gen.nVer;

      connect(port_a, borHolSeg[1].port_a1) annotation (Line(
          points={{-100,5.55112e-016},{-52,5.55112e-016},{-52,6},{-10,6}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(port_b, borHolSeg[1].port_b2) annotation (Line(
          points={{100,5.55112e-016},{28,5.55112e-016},{28,-40},{-32,-40},{-32,-6},{
              -10,-6}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(borHolSeg[gen.nVer].port_b1, borHolSeg[gen.nVer].port_a2) annotation (
         Line(
          points={{8,6},{18,6},{18,-6},{8,-6}},
          color={0,127,255},
          smooth=Smooth.None));
      for i in 1:gen.nVer - 1 loop
        connect(borHolSeg[i].port_b1, borHolSeg[i + 1].port_a1) annotation (Line(
            points={{10,6},{10,20},{-10,20},{-10,6}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(borHolSeg[i].port_a2, borHolSeg[i + 1].port_b2) annotation (Line(
            points={{10,-6},{10,-20},{-10,-20},{-10,-6}},
            color={0,127,255},
            smooth=Smooth.None));
        if use_TWall then
          connect(TWall, borHolSeg[i].TWall) annotation (Line(points={{0,102},{
                  0,56},{0,9.4},{0.2,9.4}},                                                   color={0,0,127}));
        end if;
      end for;
      connect(TWall, borHolSeg[gen.nVer].TWall) annotation (Line(points={{0,110},{0,
              12}},                                                                              color={0,0,127}));

      annotation (
        Dialog(group="Borehole"),
        Dialog(group="Borehole"),
        defaultComponentName="borehole",
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2},
            initialScale=0.5), graphics={Rectangle(
              extent={{-48,96},{48,84}},
              lineColor={0,0,255},
              fillColor={255,255,170},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2},
            initialScale=0.5), graphics={Text(
              extent={{60,72},{84,58}},
              lineColor={0,0,255},
              textString="")}),
        Documentation(info="<html>
<p>
Model of a single U-tube borehole heat exchanger. 
The borehole heat exchanger is vertically discretized into <i>n<sub>seg</sub></i>
elements of height <i>h=h<sub>Bor</sub>&frasl;n<sub>seg</sub></i>.
each final segment contains a model for the heat transfer in the borehole, 
for heat transfer in the soil and for the far-field boundary condition.
</p>
<p>
The heat transfer in the borehole is computed using a convective heat transfer coefficient
that depends on the fluid velocity, a heat resistance between the two pipes, and
a heat resistance between the pipes and the circumference of the borehole.
The heat capacity of the fluid, and the heat capacity of the grout, is taken into account.
All thermal mass is assumed to be at the two bulk temperatures of the down-flowing 
and up-flowing fluid.
</p>
<p>
The heat transfer in the soil is computed using transient heat conduction in cylindrical
coordinates for the spatial domain <i>r<sub>bor</sub> &le; r &le; r<sub>ext</sub></i>. 
In the radial direction, the spatial domain is discretized into 
<i>n<sub>hor</sub></i> segments with uniform material properties.
Thermal properties can be specified separately for each final horizontal layer.
The vertical heat flow is assumed to be zero, and there is assumed to be 
no ground water flow. 
</p>
<p>
The far-field temperature, i.e., the temperature at the radius 
<i>r<sub>ext</sub></i>, is kept constant because this model is only use to compute the short-term
temperature response of the borehole.
</p>

<h4>Implementation</h4>
<p>
each final horizontal layer is modeled using an instance of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.BoreHoleSegmentFourPort\">
IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.BoreHoleSegmentFourPort</a>.
This model is composed of the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.SingleUTubeInternalHEX\">
IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.SingleUTubeInternalHEX</a> which computes
the heat transfer in the pipes and the borehole filling, and
of the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer\">
IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer</a> which computes
the heat transfer in the soil.
</p>
</html>",     revisions="<html>
<ul>
<li>
July 2014, by Damien Picard:<br>
First implementation.
</li>
</ul>
</html>"));
    end SingleBoreHoleUTube_SoC;

    model SingleBoreHolesInSerie_SoC
      "Single or double U-tube borehole heat exchanger model. If more than one borehole is given, they are assumed to be connected in series"
      extends
        IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.Interface.PartialSingleBoreholeSerie(
         redeclare StateOfCharge.BaseClasses.SingleBoreHoleUTube_SoC
          borHol);

    parameter Real test=0;

    equation
      assert(gen.singleUTube, "This borefield model is for single U-Tube configuration but you chose double U-Tube configuration in the general borefield record.");

      annotation (
        Dialog(group="Borehole"),
        Dialog(group="Borehole"),
        defaultComponentName="borehole",
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2},
            initialScale=0.5), graphics={Rectangle(
              extent={{-30,62},{34,46}},
              lineColor={0,0,255},
              fillColor={255,255,170},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2},
            initialScale=0.5), graphics={Text(
              extent={{60,72},{84,58}},
              lineColor={0,0,255},
              textString=""),Text(
              extent={{50,-32},{90,-38}},
              lineColor={0,0,255},
              textString="")}),
        Documentation(info="<html>
<p>
Model of a single U-tube borehole heat exchanger. 
The borehole heat exchanger is vertically discretized into <i>n<sub>seg</sub></i>
elements of height <i>h=h<sub>Bor</sub>&frasl;n<sub>seg</sub></i>.
segment contains a model for the heat transfer in the borehole, 
for heat transfer in the soil and for the far-field boundary condition.
</p>
<p>
The heat transfer in the borehole is computed using a convective heat transfer coefficient
that depends on the fluid velocity, a heat resistance between the two pipes, and
a heat resistance between the pipes and the circumference of the borehole.
The heat capacity of the fluid, and the heat capacity of the grout, is taken into account.
All thermal mass is assumed to be at the two bulk temperatures of the down-flowing 
and up-flowing fluid.
</p>
<p>
The heat transfer in the soil is computed using transient heat conduction in cylindrical
coordinates for the spatial domain <i>r<sub>bor</sub> &le; r &le; r<sub>ext</sub></i>. 
In the radial direction, the spatial domain is discretized into 
<i>n<sub>hor</sub></i> segments with uniform material properties.
Thermal properties can be specified separately for horizontal layer.
The vertical heat flow is assumed to be zero, and there is assumed to be 
no ground water flow. 
</p>
<p>
The far-field temperature, i.e., the temperature at the radius 
<i>r<sub>ext</sub></i>, is computed using a power-series solution
to a line-source heat transfer problem. This temperature boundary condition
is updated every <i>t<sub>sample</sub></i> seconds.
</p>
<p>
The initial far-field temperature <i>T<sub>ext,start</sub></i>, which
is the temperature of the soil at a radius <i>r<sub>ext</sub></i>,
is computed 
as a function of the depth <i>z &gt; 0</i>. 
For a depth between <i>0 &le; z &le; z<sub>0</sub></i>, the temperature
is set to <i>T<sub>ext,0,start</sub></i>. 
The value of <i>z<sub>0</sub></i> is a parameter with a default of 10 meters.
However, there is large variability in the depth where the undisturbed soil temperature
starts.
For a depth of <i>z<sub>0</sub> &le; z &le; h<sub>bor</sub></i>,
the temperature is computed as
</p>
<p align=\"center\" style=\"font-style:italic;\">
  T<sup>i</sup><sub>ext,start</sub> = T<sub>ext,0,start</sub> + (z<sup>i</sup> - z<sub>0</sub>)  dT &frasl; dz
</p>
with <i>i &isin; {1, ..., n<sub>ver</sub>}</i>,
where the temperature gradient <i>dT &frasl; dz &ge; 0</i> is a parameter.
As with <i>z<sub>0</sub></i>, there is large variability in 
<i>dT &frasl; dz &ge; 0</i>. The default value is set to <i>1</i> Kelvin per 100 meters.
For the temperature of the grout, the same equations are applied, with
<i>T<sub>ext,0,start</sub></i> replaced with
<i>T<sub>fil,0,start</sub></i>, and 
<i>T<sup>i</sup><sub>ext,start</sub></i> replaced with
<i>T<sup>i</sup><sub>fil,start</sub></i>. 
The default setting uses the same temperature for the soil and the filling material.
</p>
<h4>Implementation</h4>
<p>
horizontal layer is modeled using an instance of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.Boreholes.BaseClasses.BoreholeSegment\">
IDEAS.HeatExchangers.Fluid.Boreholes.BaseClasses.BoreholeSegment</a>.
This model is composed of the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.Boreholes.BaseClasses.HexInternalElement\">
IDEAS.Fluid.HeatExchangers.Boreholes.BaseClasses.HexInternalElement</a> which computes
the heat transfer in the pipes and the borehole filling,
of the model
<a href=\"modelica://IDEAS.HeatTransfer.Conduction.SingleLayerCylinder\">
IDEAS.HeatTransfer.Conduction.SingleLayerCylinder</a> which computes
the heat transfer in the soil, and
of the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.Boreholes.BaseClasses.TemperatureBoundaryCondition\">
IDEAS.Fluid.HeatExchangers.Boreholes.BaseClasses.TemperatureBoundaryCondition</a> which computes
the far-field temperature boundary condition.
</p>
</html>",     revisions="<html>
<ul>
<li>
August 2011, by Pierre Vigouroux:<br>
First implementation.
</li>
</ul>
</html>"));
    end SingleBoreHolesInSerie_SoC;
  end BaseClasses;

  model MultipleBoreHolesUTube_SoC
    "Borefield model using single U-tube borehole heat exchanger configuration."

    // Medium in borefield
    extends
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Interfaces.partial_multipleBoreHoles;
        replaceable StateOfCharge.BaseClasses.SingleBoreHolesInSerie_SoC
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
      annotation (Line(points={{-60,0},{-36,0},{-36,1.55431e-015},{-13,
            1.55431e-015}},                              color={0,127,255}));
    connect(borHolSer.port_b, massFlowRateMultiplier1.port_a)
      annotation (Line(points={{11,-1.33227e-015},{36,-1.33227e-015},{36,0},{60,
            0}},                                      color={0,127,255}));
    connect(TWall_val.y, borHolSer.TWall)
      annotation (Line(points={{-18.9,40},{-0.76,40},{-0.76,12.22}},
                                                              color={0,0,127}));
    annotation (
      experiment(StopTime=70000, __Dymola_NumberOfIntervals=50),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}),
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
            fillPattern=FillPattern.Forward),
          Rectangle(
            extent={{-100,78},{100,66}},
            lineColor={0,0,255},
            fillColor={255,255,170},
            fillPattern=FillPattern.Solid)}),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics),
                      Documentation(info="<html>
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
  end MultipleBoreHolesUTube_SoC;

  model Example01_SoC
    "Model of a borefield in a 8x1 boreholes line configuration and a constant heat injection rate"

    extends Modelica.Icons.Example;

    parameter Modelica.SIunits.Temperature T_start = bfData.gen.T_start;
    package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

    replaceable parameter
      IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Data.BorefieldData.SandStone_Bentonite_c8x1_h110_b5_d3600_T283
      bfData annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
    parameter Integer lenSim=3600*24*366 "length of the simulation";

    replaceable MultipleBoreHolesUTube_SoC
      borFie(
      lenSim=lenSim,
      redeclare package Medium = Medium,
      bfData=bfData,
      T_start=T_start) "borefield"
      annotation (Placement(transformation(extent={{-20,-60},{20,-20}})));
    Modelica.Blocks.Sources.Step load(height=1, startTime=36000)
      "load for the borefield"
      annotation (Placement(transformation(extent={{26,-18},{40,-4}})));

    IDEAS.Fluid.HeatExchangers.HeaterCooler_u hea(
      redeclare package Medium = Medium,
      dp_nominal=10000,
      show_T=true,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      m_flow_nominal=bfData.m_flow_nominal,
      m_flow(start=bfData.m_flow_nominal),
      Q_flow_nominal=bfData.gen.q_ste*bfData.gen.nbBh*bfData.gen.hBor,
      T_start=T_start,
      p_start=100000)
      annotation (Placement(transformation(extent={{30,22},{10,2}})));
    Modelica.Fluid.Sources.Boundary_pT boundary(          redeclare package
        Medium = Medium, nPorts=1)
      annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem_out(
      redeclare package Medium = Medium,
      m_flow_nominal=bfData.m_flow_nominal,
      T_start=T_start)
      annotation (Placement(transformation(extent={{38,-50},{58,-30}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow pum(
      redeclare package Medium = Medium,
      dynamicBalance=false,
      m_flow_nominal=bfData.m_flow_nominal,
      T_start=T_start,
      motorCooledByFluid=false,
      addPowerToMedium=false,
      filteredSpeed=false)
      annotation (Placement(transformation(extent={{-16,22},{-36,2}})));
    Modelica.Blocks.Sources.Constant mFlo(k=bfData.m_flow_nominal)
      annotation (Placement(transformation(extent={{-60,-18},{-48,-6}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem_in(
      redeclare package Medium = Medium,
      m_flow_nominal=bfData.m_flow_nominal,
      T_start=T_start)
      annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  equation
    connect(load.y, hea.u) annotation (Line(
        points={{40.7,-11},{52,-11},{52,6},{32,6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hea.port_a, senTem_out.port_b) annotation (Line(
        points={{30,12},{70,12},{70,-40},{58,-40}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(senTem_out.port_a, borFie.port_b) annotation (Line(
        points={{38,-40},{20,-40}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(mFlo.y, pum.m_flow_in) annotation (Line(
        points={{-47.4,-12},{-25.8,-12},{-25.8,0}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pum.port_a, hea.port_b) annotation (Line(
        points={{-16,12},{10,12}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(boundary.ports[1], pum.port_b) annotation (Line(
        points={{-40,50},{-36,50},{-36,12}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pum.port_b, senTem_in.port_a) annotation (Line(
        points={{-36,12},{-78,12},{-78,-40},{-60,-40}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(senTem_in.port_b, borFie.port_a) annotation (Line(
        points={{-40,-40},{-20,-40}},
        color={0,127,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {100,100}}),
                      graphics),
      experiment(StopTime=1.7e+006, __Dymola_NumberOfIntervals=100),
      __Dymola_experimentSetupOutput,
      Documentation(info="<html>
</html>",   revisions="<html>
<ul>
<li>
July 2014, by Damien Picard:<br>
First implementation.
</li>
</ul>
</html>"));
  end Example01_SoC;
  annotation (uses(Modelica(version="3.2.1"), IDEAS(version="0.2")));
end StateOfCharge;
