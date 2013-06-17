FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/dnepr_quadro_control/msg"
  "../src/dnepr_quadro_control/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/dnepr_quadro_control/msg/__init__.py"
  "../src/dnepr_quadro_control/msg/_RawArduinoControl.py"
  "../src/dnepr_quadro_control/msg/_FlightDirection.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
