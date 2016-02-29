within IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Examples;
model borefied8x1_2UTubeParallel_2Series
  extends
    IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Examples.borefield8x1
    ( redeclare
      Data.BorefieldData.SandStone_Bentonite_c8x1_h110_b5_d600_T283_2UTubeParallel_2Series
                                                                                                          bfData(
      loaPerMBor_nominal=40,
      PThe_nominal=gen.hBor*gen.nbBh*loaPerMBor_nominal,
      m_flow_nominal=2),
      redeclare MultipleBoreHoles2UTube borFie(m_flow_nominal=bfData.m_flow_nominal,
        dp_nominal=0));
  annotation (experiment(StopTime=3.1536e+007), __Dymola_experimentSetupOutput(
        events=false));
end borefied8x1_2UTubeParallel_2Series;
