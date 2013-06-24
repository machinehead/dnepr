FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/kinect_utils/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/kinect_utils/msg/__init__.py"
  "../src/kinect_utils/msg/_flightCommand.py"
  "../src/kinect_utils/msg/_kinectPose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
