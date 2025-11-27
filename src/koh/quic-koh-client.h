//
// Created by loapp on 25. 8. 18.
//

#ifndef UDP_KOH_CLIENT_H
#define UDP_KOH_CLIENT_H
#include "kohTag.h"

#endif //UDP_KOH_CLIENT_H
/*
* Copyright (c) 2007,2008,2009 INRIA, UDCAST
 */



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

    class QuicKohClient : public Application
    {
    public:
        static TypeId GetTypeId();

        QuicKohClient();
        ~QuicKohClient() override;

        uint64_t GetTotalTx() const;

        void changeInterface();
        void setInterface(Ptr<NetDevice> uu, Ptr<NetDevice> sl);
        // [수정] RSU의 Sidelink IP 주소(Next Hop)를 받기 위한 파라미터 추가
        void SetTag(KohTag temp);
        void SelectInterface(uint32_t i);
        uint16_t GetUeId();
        void clearCount();
        uint64_t GetTotalSent();
        Callback<void, KStats> m_KCallback;

    private:
        void StartApplication() override;
        void StopApplication() override;
        void SendSl();
        void SendUu();
        void SelectInterface(Ptr<Socket> socket);
        void HandleRecv(Ptr<Socket> socket);
        void SendSplit();

        TracedCallback<Ptr<const Packet>> m_txTrace;
        TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_txTraceWithAddresses;

        uint32_t m_count;
        Time m_interval;
        uint32_t m_size;

        uint32_t m_sentUu;//1동안 얼마나 받았나
        uint32_t m_sentSl;//1동안 얼마나 받았나
        uint32_t m_sentUuSync;//메인코드로 전송
        uint32_t m_sentSlSync;//메인코드로 전송
        uint32_t m_totalSent;//전체 보낸것
        uint64_t m_seqSl;
        uint64_t m_seqUu;
        Time m_intervalSl;
        Time m_intervalUu;


        uint64_t m_totalTx;
        Ptr<Socket> m_uuSocket;
        Ptr<Socket> m_slSocket;
        Ptr<Socket> m_sendSocket;
        Ptr<Socket> m_recvSocket;
        Ptr<Socket> m_rsrpSocket;
        Address m_slServerAddress;
        uint16_t m_slServerPort;
        Address m_uuServerAddress;
        uint16_t m_uuServerPort;
        Ptr<NetDevice> m_devUu, m_devSl;


        bool m_slSocketConnected;
        // uint8_t m_tos;
        EventId m_sendUuEvent;
        EventId m_sendSlEvent;

        uint32_t m_numStreams;
        uint32_t m_lastUsedStream;

        void SendBurst();
        void ScheduleNextPacketInBurst(uint32_t count);
        void SyncSent();

        uint32_t m_burstPacketCount;
        Time     m_burstInterval;
        KohTag tagSl;
        // KohTag tagUu;
        uint32_t m_packetCounter;

#ifdef NS3_LOG_ENABLE
        std::string m_peerAddressString;
#endif
    };

} // namespace ns3

