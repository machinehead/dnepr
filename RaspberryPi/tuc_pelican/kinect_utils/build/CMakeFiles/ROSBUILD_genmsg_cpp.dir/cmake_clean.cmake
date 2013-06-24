FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/kinect_utils/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/kinect_utils/flightCommand.h"
  "../msg_gen/cpp/include/kinect_utils/kinectPose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
