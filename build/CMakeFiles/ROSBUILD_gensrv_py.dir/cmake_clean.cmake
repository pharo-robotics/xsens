FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/xsens/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/xsens/srv/__init__.py"
  "../src/xsens/srv/_Calibrate.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
