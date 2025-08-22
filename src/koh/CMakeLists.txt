build_lib(
  LIBNAME koh
  SOURCE_FILES
    udp-koh-client.cc
    udp-koh-server.cc
        quic-koh-client.cc
        quic-koh-server.cc
  HEADER_FILES
        udp-koh-client.h
        udp-koh-server.h
        quic-koh-client.h
        quic-koh-server.h
        LIBRARIES_TO_LINK ${libinternet}
)
