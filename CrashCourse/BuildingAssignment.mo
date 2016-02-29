within ;
package BuildingAssignment
  model Building

    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RSlab1(R=0.016)
      annotation (Placement(transformation(
          extent={{-9,-9},{9,9}},
          rotation=90,
          origin={79,61})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RSlab2(R=0.016)
      annotation (Placement(transformation(
          extent={{-9,-9},{9,9}},
          rotation=90,
          origin={79,27})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RWall(R=0.00806)
      annotation (Placement(transformation(
          extent={{-9,-9},{9,9}},
          rotation=0,
          origin={53,77})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CZone(C=2.4096E+8)
      annotation (Placement(transformation(extent={{74,86},{86,98}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro2(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={59,-19})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro1(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={85,-19})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CGro(C=2.52E+8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=180,
          origin={72,-32})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CSlab(C=3.36E+8)
      annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={94,44})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro3(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={13,-19})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro4(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={39,-19})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CGro1(C=2.52E+8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=180,
          origin={26,-32})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro5(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-33,-19})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro6(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-7,-19})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CGro2(C=2.52E+8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=180,
          origin={-20,-32})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro7(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-79,-19})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro8(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-53,-19})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CGro3(C=2.52E+8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=180,
          origin={-66,-32})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro9(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-123,-19})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro10(R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-97,-19})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CGro4(C=2.52E+8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=180,
          origin={-110,-32})));
    Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixedHeatFlow
      annotation (Placement(transformation(extent={{-78,52},{-58,72}})));
    Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={-152,-18})));

    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature
      annotation (Placement(transformation(extent={{-2,66},{18,86}})));
  equation
    connect(RGro2.port_b, RGro1.port_a) annotation (Line(
        points={{64,-19},{80,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CGro.port, RGro2.port_b) annotation (Line(
        points={{72,-28},{72,-19},{64,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro3.port_b, RGro4.port_a) annotation (Line(
        points={{18,-19},{34,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CGro1.port, RGro3.port_b) annotation (Line(
        points={{26,-28},{26,-19},{18,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro5.port_b, RGro6.port_a) annotation (Line(
        points={{-28,-19},{-12,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CGro2.port, RGro5.port_b) annotation (Line(
        points={{-20,-28},{-20,-19},{-28,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro7.port_b, RGro8.port_a) annotation (Line(
        points={{-74,-19},{-58,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CGro3.port, RGro7.port_b) annotation (Line(
        points={{-66,-28},{-66,-19},{-74,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro9.port_b, RGro10.port_a) annotation (Line(
        points={{-118,-19},{-102,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CGro4.port, RGro9.port_b) annotation (Line(
        points={{-110,-28},{-110,-19},{-118,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro1.port_b, RSlab2.port_a) annotation (Line(
        points={{90,-19},{84,-19},{84,18},{79,18}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RSlab1.port_b, CZone.port) annotation (Line(
        points={{79,70},{80,70},{80,86}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RWall.port_b, CZone.port) annotation (Line(
        points={{62,77},{72,77},{72,76},{80,76},{80,86}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CSlab.port, RSlab1.port_a) annotation (Line(
        points={{88,44},{80,44},{79,52}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RSlab1.port_a, RSlab2.port_b) annotation (Line(
        points={{79,52},{76,52},{76,36},{79,36}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro2.port_a, RGro4.port_b) annotation (Line(
        points={{54,-19},{50,-19},{50,-19},{44,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro3.port_a, RGro6.port_b) annotation (Line(
        points={{8,-19},{4,-19},{4,-19},{-2,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro5.port_a, RGro8.port_b) annotation (Line(
        points={{-38,-19},{-44,-19},{-44,-19},{-48,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro7.port_a, RGro10.port_b) annotation (Line(
        points={{-84,-19},{-88,-19},{-88,-19},{-92,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(temperatureSensor.port, RGro9.port_a) annotation (Line(
        points={{-146,-18},{-138,-18},{-138,-19},{-128,-19}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(prescribedTemperature.port, RWall.port_a) annotation (Line(
        points={{18,76},{32,76},{32,77},{44,77}},
        color={191,0,0},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),      graphics));
  end Building;

  model BuildingNew

    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RSlab1(R=0.016)
      annotation (Placement(transformation(
          extent={{-9,-9},{9,9}},
          rotation=90,
          origin={55,39})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RSlab2(R=0.016)
      annotation (Placement(transformation(
          extent={{-9,-9},{9,9}},
          rotation=90,
          origin={55,-5})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RWall(R=0.00806)
      annotation (Placement(transformation(
          extent={{-9,-9},{9,9}},
          rotation=0,
          origin={-15,75})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CZone(C=2.4096E+8)
      annotation (Placement(transformation(extent={{50,86},{62,98}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro2[5](R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-27,-53})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor RGro1[5](R=0.033)
      annotation (Placement(transformation(
          extent={{-5,-5},{5,5}},
          rotation=0,
          origin={-1,-53})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CGro[5](C=2.52E+8)
      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=180,
          origin={-14,-66})));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor CSlab(C=3.36E+8)
      annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={86,14})));
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TGroIni(T(
          displayUnit="K") = 283.15)
      annotation (Placement(transformation(extent={{-72,-64},{-52,-44}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TAmb
      annotation (Placement(transformation(extent={{-84,66},{-64,86}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow QSol
      annotation (Placement(transformation(extent={{-6,44},{14,64}})));
    Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor TSlab annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=180,
          origin={18,18})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{80,66},{100,86}})));
  equation
    connect(RSlab1.port_b, CZone.port) annotation (Line(
        points={{55,48},{56,48},{56,86}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro1[1].port_b, RSlab2.port_a) annotation (Line(
        points={{4,-53},{28,-53},{28,-16},{55,-16},{55,-14}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro2[5].port_a, TGroIni.port) annotation (Line(
        points={{-32,-53},{-42,-53},{-42,-54},{-52,-54}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(QSol.port, CZone.port) annotation (Line(
        points={{14,54},{48,54},{48,70},{56,70},{56,86}},
        color={191,0,0},
        smooth=Smooth.None));
  TAmb.T=10*cos(2*3.14*time*3*10^(-8))+276.15;

  QSol.Q_flow= floor(cos(2* Modelica.Constants.pi * time /86400)+1)*5000*cos(2* Modelica.Constants.pi * time /86400);
    connect(TAmb.port, RWall.port_a) annotation (Line(
        points={{-64,76},{-48,76},{-48,75},{-24,75}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RWall.port_b, CZone.port) annotation (Line(
        points={{-6,75},{24,75},{24,74},{56,74},{56,86}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RSlab1.port_a, RSlab2.port_b) annotation (Line(
        points={{55,30},{55,30},{55,4}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CSlab.port, RSlab2.port_b) annotation (Line(
        points={{80,14},{56,14},{55,4}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(TSlab.port, RSlab2.port_b) annotation (Line(
        points={{24,18},{56,18},{55,4}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(temperatureSensor.port, CZone.port) annotation (Line(
        points={{80,76},{56,76},{56,86}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(RGro2.port_b, RGro1.port_a) annotation (Line(
        points={{-22,-53},{-14,-53},{-14,-53},{-6,-53}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(CGro.port, RGro2.port_b) annotation (Line(
        points={{-14,-62},{-14,-53},{-22,-53}},
        color={191,0,0},
        smooth=Smooth.None));

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), __Dymola_Commands(file(
            ensureSimulated=true) = "*.mos"));
  end BuildingNew;
  annotation (uses(Modelica(version="3.2.1")));
  model OneDimensionalHeatTransfer

  type Temperature=Real(unit="K",min=0);
  type ConvectionCoefficient=Real(unit="W/K", min=0);
  type ConductionCoefficient=Real(unit="W.m-1.K-1", min=0);
  type Mass=Real(unit="kg", min=0);
  type SpecificHeat=Real(unit="J/(K.kg)", min=0);
  type Density=Real(unit="kg/m3", min=0);
  type Area=Real(unit="m2");
  type Volume=Real(unit="m3");
  type Length=Real(unit="m", min=0);
  type Radius=Real(unit="m", min=0);

  constant Real pi = 3.14159;

  parameter Integer n=10;
  parameter Length L=1.0;
  parameter Radius R=0.1;
  parameter Density rho=2.0;
  parameter ConvectionCoefficient h=2.0;
  parameter ConductionCoefficient k=10;
  parameter SpecificHeat C=10.0;
  parameter Temperature Tamb=300 "Ambient temperature";

  parameter Area A = pi*R^2;
  parameter Volume V = A*L/n;

  Temperature T[n];

  initial equation
    T = linspace(200,300,n);

  equation
  rho*V*C*der(T[1]) = -h*(T[1]-Tamb)-k*A*(T[1]-T[2])/(L/n);
  for i in 2:(n-1) loop
    rho*V*C*der(T[i]) = -k*A*(T[i]-T[i-1])/(L/n)-k*A*(T[i]-T[i+1])/(L/n);
  end for;
  rho*V*C*der(T[end]) = -h*(T[end]-Tamb)-k*A*(T[end]-T[end-1]);

  end OneDimensionalHeatTransfer;
end BuildingAssignment;
