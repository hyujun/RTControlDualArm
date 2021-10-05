file(REMOVE_RECURSE
  "libNetworkd.a"
  "libNetworkd.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Network.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
