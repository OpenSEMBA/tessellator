Original port: https://github.com/microsoft/vcpkg/tree/fedfb6ae868887a13f8a0ba3b29603bd7eb9f118/ports/libaec

A different download url is used to avoid connection errors.

libaec provides CMake targets:
``` Cmake
  find_package(libaec CONFIG REQUIRED)
  # libaec API
  target_link_libraries(main PRIVATE libaec::aec)
  # szip compatible API
  target_link_libraries(main PRIVATE libaec::sz)
```