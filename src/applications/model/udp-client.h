/*
* Copyright (c) 2007,2008,2009 INRIA, UDCAST
 */

#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/internet-module.h"
#include "ns3/ptr.h"
#include <ns3/traced-callback.h>

namespace ns3
{

    class Socket;
    class Packet;

    class UdpClient : public Application
    {
    public:
        static TypeId GetTypeId();

        UdpClient();
        ~UdpClient() override;

        uint64_t GetTotalTx() const;

        void changeInterface();
        void setInterface(Ptr<NetDevice> uu, Ptr<NetDevice> sl);
        // [수정] RSU의 Sidelink IP 주소(Next Hop)를 받기 위한 파라미터 추가

    private:
        void StartApplication() override;
        void StopApplication() override;
        void Send();
        void SelectInterface(Ptr<Socket> socket);
        void HandleRecv(Ptr<Socket> socket);

        TracedCallback<Ptr<const Packet>> m_txTrace;
        TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_txTraceWithAddresses;

        uint32_t m_count;
        Time m_interval;
        uint32_t m_size;

        uint32_t m_sent;
        uint64_t m_totalTx;
        Ptr<Socket> m_uuSocket;
        Ptr<Socket> m_slSocket;
        Ptr<Socket> m_sendSocket;
        Ptr<Socket> m_recvSocket;
        Address m_slServerAddress;
        uint16_t m_slServerPort;
        uint16_t m_recvPort;
        Address m_uuServerAddress;
        uint16_t m_uuServerPort;
        Ptr<NetDevice> m_devUu, m_devSl;


        uint8_t m_tos;
        EventId m_sendEvent;

#ifdef NS3_LOG_ENABLE
        std::string m_peerAddressString;
#endif
    };

} // namespace ns3

#endif /* UDP_CLIENT_H */