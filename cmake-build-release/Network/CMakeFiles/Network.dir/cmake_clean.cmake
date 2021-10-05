file(REMOVE_RECURSE
  "libNetwork.a"
  "libNetwork.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Network.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
