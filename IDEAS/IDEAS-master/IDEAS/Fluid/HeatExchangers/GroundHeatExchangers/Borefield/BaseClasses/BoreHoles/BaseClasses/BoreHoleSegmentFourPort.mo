within IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.BaseClasses;
model BoreHoleSegmentFourPort "Vertical segment of a borehole"
  extends Interface.PartialBoreHoleSegment;

  extends IDEAS.Fluid.Interfaces.PartialFourPortInterface(
    redeclare final package Medium1 = Medium,
    redeclare final package Medium2 = Medium,
    final m1_flow_small=gen.m_flow_small,
    final m2_flow_small=gen.m_flow_small,
    final allowFlowReversal1=gen.allowFlowReversal,
    final allowFlowReversal2=gen.allowFlowReversal);

  StateOfCharge.IntHEX_SoC intHEX_SoC
    annotation (Placement(transformation(extent={{-236,38},{-216,60}})));
  StateOfCharge.Example01 example01_1
    annotation (Placement(transformation(extent={{-258,64},{-238,84}})));
  StateOfCharge.IntHEX_SoC intHEX_SoC1
    annotation (Placement(transformation(extent={{-70,-20},{-50,2}})));
  StateOfCharge.SoilLay_SoC soilLay_SoC annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-38,22})));
equation
  if not use_TWall then
  else
  end if;
  connect(intHEX_SoC1.port_b1, port_b1) annotation (Line(
      points={{-50,-2},{28,-2},{28,60},{100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(intHEX_SoC1.port_a2, port_a2) annotation (Line(
      points={{-50,-14},{26,-14},{26,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(intHEX_SoC1.port_b2, port_b2) annotation (Line(
      points={{-70,-14},{-86,-14},{-86,-60},{-100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(intHEX_SoC1.port_a1, port_a1) annotation (Line(
      points={{-70,-2},{-84,-2},{-84,60},{-100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(soilLay_SoC.port_a, intHEX_SoC1.port) annotation (Line(
      points={{-38,12},{-48,12},{-48,2},{-60,2}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(soilLay_SoC.port_b, TBouCon.port) annotation (Line(
      points={{-38,32},{-50,32},{-50,60},{-60,60}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}}),
                    graphics),
    Icon(graphics={
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
          fillPattern=FillPattern.Backward)}),
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
</html>", revisions="<html>
<ul>
<li>
July 2014, by Damien Picard:<br>
First implementation.
</li>
</ul>
</html>"));
end BoreHoleSegmentFourPort;
