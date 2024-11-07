# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/v5.1.4/esp-idf/components/bootloader/subproject"
  "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader"
  "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix"
  "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix/tmp"
  "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix/src"
  "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Jigisha/ESP-IDF/bme688_NW/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
