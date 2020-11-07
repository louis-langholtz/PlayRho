configure_file(Doxyfile.in Doxyfile @ONLY)

add_custom_target(Documentation ALL "${DOXYGEN_EXECUTABLE}"
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Documentation"
  COMMENT "Generating HTML documentation" VERBATIM)

install(
  DIRECTORY "${CMAKE_BINARY_DIR}/Documentation/html"
  DESTINATION "${CMAKE_INSTALL_DOCDIR}"
)
