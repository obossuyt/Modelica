within IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.BaseClasses.BoreHoles.Interface;
partial model PartialTWall
    parameter Boolean use_TWall = false
    "Set to true if an input is used for the borehole wall temperature instead of computing it wihtin the model."
    annotation (Dialog(tab="Advanced",group="Model use"));

    Modelica.Blocks.Interfaces.RealInput TWall(unit="K",displayUnit="degC") if use_TWall
    annotation (Placement(transformation(extent={{20,-20},{-20,20}},
        rotation=90,
        origin={0,102}), iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={2,94})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
end PartialTWall;
