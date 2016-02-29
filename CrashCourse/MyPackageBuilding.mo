within ;
package MyPackageBuilding
  model MyBuilding
    extends IDEAS.Interfaces.Building(redeclare package Medium=
          Lib.Example_IDEAS.Data.Medium);
  end MyBuilding;
  annotation (uses(IDEAS(version="0.2"), Modelica(version="3.2.1")));
end MyPackageBuilding;
