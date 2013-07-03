FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/dnepr_quadro_control/msg"
  "../src/dnepr_quadro_control/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/dnepr_quadro_control/CameraCrossFinder.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
