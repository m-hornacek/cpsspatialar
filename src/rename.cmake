if(EXISTS "${InputLibPath}")
  execute_process(COMMAND ${CMAKE_COMMAND}
  -E copy ${InputLibPath} ${OutputLibPath})
endif()