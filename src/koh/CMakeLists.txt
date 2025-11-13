build_lib(
  LIBNAME koh
  SOURCE_FILES
        udp-koh-client.cc
        udp-koh-server.cc
        quic-koh-client.cc
        quic-koh-server.cc
        v2x-kpi.cc
  HEADER_FILES
        udp-koh-client.h
        udp-koh-server.h
        quic-koh-client.h
        quic-koh-server.h
        kohTag.h
        v2x-kpi.h
        LIBRARIES_TO_LINK ${libinternet}
)
