within ;
package TEST
  model test1
    Modelica.Thermal.HeatTransfer.Components.ThermalConductor thermalConductor(
        G=0.005) annotation (Placement(transformation(extent={{8,20},{28,40}})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heatCapacitor(C=5)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-18,16})));
    Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow(Q_flow=
          1000)
      annotation (Placement(transformation(extent={{-54,34},{-34,54}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{46,20},{66,40}})));
  equation
    connect(thermalConductor.port_a, heatCapacitor.port) annotation (Line(
        points={{8,30},{-6,30},{-6,26},{-18,26}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(fixedHeatFlow.port, heatCapacitor.port) annotation (Line(
        points={{-34,44},{-26,44},{-26,26},{-18,26}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalConductor.port_b, temperatureSensor.port) annotation (Line(
        points={{28,30},{46,30}},
        color={191,0,0},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
              -100,-100},{100,100}}), graphics));
  end test1;
  annotation (uses(Modelica(version="3.2.1"),
      IDEAS(version="0.2"),
      Buildings(version="2.1.0")));
end TEST;
