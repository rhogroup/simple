FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/simple/msg"
  "../src/simple/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/tests"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/tests.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
