FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/simple/msg"
  "../src/simple/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/simple/set_goal.h"
  "../srv_gen/cpp/include/simple/controller_pub2.h"
  "../srv_gen/cpp/include/simple/setGoal.h"
  "../srv_gen/cpp/include/simple/controller_pub.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
