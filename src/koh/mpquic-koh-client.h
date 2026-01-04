//
// Created by loapp on 25. 8. 18.
//

#ifndef MPQUIC_KOH_CLIENT_H
#define MPQUIC_KOH_CLIENT_H
#include "kohTag.h"

/*
* Copyright (c) 2007,2008,2009 INRIA, UDCAST
 */



#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/internet-module.h"
#include "ns3/ptr.h"
#include <ns3/traced-callback.h>
#include <fstream>
#include <vector>
#include <string>



// static uint32_t g_currentFrameIndex = 0;
namespace ns3
{
static std::vector<frameData> g_videoFrames;
static std::vector<frameData>::iterator g_videoFramesIterator;

    class Socket;
    class Packet;

    class MPQuicKohClient : public Application
    {
    public:
        static TypeId GetTypeId();

        MPQuicKohClient();
        ~MPQuicKohClient() override;

        uint64_t GetTotalTx() const;

        void SetTag(KohTag temp);
        uint16_t GetUeId();
        void clearCount();
        uint64_t GetTotalSent();
        void LoadVideoData(std::string filename);
        Callback<void, KStats> m_KCallback;


    private:
        void StartApplication() override;
        void StopApplication() override;
        void Send();
        void SendFrame(frameData fd);
        void ReadVideoData();
        void HandleRecv(Ptr<Socket> socket);


        TracedCallback<Ptr<const Packet>> m_txTrace;
        TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_txTraceWithAddresses;

        uint32_t m_count;
        Time m_interval;
        uint32_t m_size;

        uint32_t m_sent;//1동안 얼마나 받았나
        uint32_t m_sentSync;//메인코드로 전송
        uint32_t m_totalSent;//전체 보낸것
        uint64_t m_seq;
        Time m_intervalReadVideoData;


        uint64_t m_totalTx;
        Ptr<Socket> m_Socket;
        Ptr<Socket> m_sendSocket;
        Ptr<Socket> m_recvSocket;
        Ptr<Socket> m_rsrpSocket;
        Address m_ServerAddress;
        uint16_t m_ServerPort;
        Ptr<NetDevice> m_devUu, m_devSl;


        // bool m_slSocketConnected;
        // uint8_t m_tos;
        EventId m_sendEvent;
        EventId m_readVideoDataEvent;

        uint32_t m_numStreams;
        uint32_t m_lastUsedStream;

        void SendBurst();
        void ScheduleNextPacketInBurst(uint32_t count);
        void SyncSent();

        uint32_t m_burstPacketCount;
        Time     m_burstInterval;
        KohTag tag;
        uint32_t m_packetCounter;
        uint16_t m_gop;

#ifdef NS3_LOG_ENABLE
        std::string m_peerAddressString;
#endif
    };
#endif
} // namespace ns3

