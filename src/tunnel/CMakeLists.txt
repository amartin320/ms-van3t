
Set(source_files

  #model/Applications/camMonitor.cc

  model/Devices/its-station.cc
  model/Devices/roadside-its-station.cc
  model/Devices/vehicle-its-station.cc

  #helper/camMonitor-helper.cc
  )

Set(header_files

  #model/Applications/camMonitor.h

  model/Devices/its-station.h
  model/Devices/roadside-its-station.h
  model/Devices/vehicle-its-station.h

  #helper/camMonitor-helper.h
)

build_lib(
  LIBNAME tunnel
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libfd-net-device}
    ${libwifi}
    ${libapplications}
    ${libmobility}
    ${libconfig-store}
    ${libstats}
    ${libinternet}
    ${libvehicle-visualizer}
    ${libtraci}
)