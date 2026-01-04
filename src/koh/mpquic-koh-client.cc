/*
 * Copyright (c) 2007,2008,2009 INRIA, UDCAST
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Amine Ismail <amine.ismail@sophia.inria.fr>
 *                      <amine.ismail@udcast.com>
 */
#include "mpquic-koh-client.h"

#include "./../applications/model/seq-ts-header.h"

#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/quic-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"

#include <cstdio>
#include <cstdlib>
#include <thread>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MPQuicKohClient");

NS_OBJECT_ENSURE_REGISTERED(MPQuicKohClient);

TypeId
MPQuicKohClient::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MPQuicKohClient")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<MPQuicKohClient>()
            .AddAttribute(
                "MaxPackets",
                "The maximum number of packets the application will send (zero means infinite)",
                UintegerValue(100),
                MakeUintegerAccessor(&MPQuicKohClient::m_count),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("Interval",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&MPQuicKohClient::m_interval),
                          MakeTimeChecker())
            .AddAttribute("IntervalReadVideoData",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&MPQuicKohClient::m_intervalReadVideoData),
                          MakeTimeChecker())
            .AddAttribute("ServerAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&MPQuicKohClient::m_ServerAddress),
                          MakeAddressChecker())
            .AddAttribute("ServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&MPQuicKohClient::m_ServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("PacketSize",
                          "Size of packets generated. The minimum packet size is 12 bytes which is "
                          "the size of the header carrying the sequence number and the time stamp.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&MPQuicKohClient::m_size),
                          MakeUintegerChecker<uint32_t>(12, 65507))
            .AddTraceSource("Tx",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&MPQuicKohClient::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("TxWithAddresses",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&MPQuicKohClient::m_txTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

MPQuicKohClient::MPQuicKohClient()
{
    NS_LOG_FUNCTION(this);
    m_sent = 0;
    m_totalTx = 0;
    m_Socket = nullptr;
    m_sendSocket = nullptr;
    m_sendEvent = EventId();
    m_lastUsedStream = 1;
    // m_slSocketConnected = false;
    m_seq = 0;
    m_packetCounter = 0;

    m_totalSent = 0;
}

MPQuicKohClient::~MPQuicKohClient()
{
    NS_LOG_FUNCTION(this);
}

void
MPQuicKohClient::StartApplication()
{
    NS_LOG_FUNCTION(this);

    // TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    // //  소켓 설정 rsrp
    // m_rsrpSocket = Socket::CreateSocket(GetNode(), tid);
    // m_rsrpSocket->BindToNetDevice(m_devSl);

    // // listening 소켓 설정
    // m_recvSocket = Socket::CreateSocket(GetNode(), TypeId(QuicSocketFactory::GetTypeId()));
    // if (m_recvSocket->Bind() == -1)
    // {
    //     NS_FATAL_ERROR("UdpRelay: Failed to bind In-Socket");
    // }

    // 소켓 설정
    m_Socket = Socket::CreateSocket(GetNode(), TypeId(QuicSocketFactory::GetTypeId()));
    m_Socket->Bind();
    m_Socket->BindToNetDevice(m_devUu); // Connect 전에 Bind
    if (Ipv6Address::IsMatchingType(m_ServerAddress))
    {
        NS_LOG_UNCOND("uu ipv6 socket connection started");
        m_Socket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_ServerAddress), m_ServerPort));
    }
    else
    {
        NS_LOG_UNCOND("uu ipv4 socket connection started");
        m_Socket->Connect(
            InetSocketAddress(Ipv4Address::ConvertFrom(m_ServerAddress), m_ServerPort));
    }

    // 수신 콜백 설정 (양방향 통신 시 필요)
    m_Socket->SetRecvCallback(MakeCallback(&MPQuicKohClient::HandleRecv, this));

    // 영상 데이터
    // m_readVideoDataEvent = Simulator::Schedule(Seconds(0.0), &MPQuicKohClient::ReadVideoData, this);

    m_sendEvent = Simulator::Schedule(Seconds(0.0), &MPQuicKohClient::Send, this);

}

void
MPQuicKohClient::HandleRecv(Ptr<Socket> socket)
{
    SyncSent();

    NS_LOG_UNCOND("MPQuicKohClient::HandleRecv");
    NS_LOG_FUNCTION(this << socket);
    Address from;
    Ptr<Packet> packet = socket->RecvFrom(from);
    uint8_t* buffer = new uint8_t[packet->GetSize() + 1];
    packet->CopyData(buffer, packet->GetSize());
    buffer[packet->GetSize()] = '\0';

    if (InetSocketAddress::IsMatchingType(from))
    {
        // NS_LOG_UNCOND("Received a " << (socket == m_slSocket ? tagSl.ueId : tagUu.ueId) << " ue "
        NS_LOG_UNCOND("Received "
                                    << packet->GetSize() << " bytes packet from"
                                    << InetSocketAddress::ConvertFrom(from).GetIpv4() << ": "
                                    << buffer);

        KStats stats;
        packet->CopyData((uint8_t*)&stats, sizeof(KStats));
        // stats.sentCount = m_sentSync; //todo: 나중에 다시 살ㄹ리셈 일단 생략
        // NS_LOG_UNCOND("UE " << stats.ueId
        //   << " RecvCount=" << stats.recvCount
        //   << " PRR=" << stats.recvCount/m_sentSync
        //   << " Latency=" << stats.avgLatency << " ms");
        m_KCallback(stats);
    }
    else if (Inet6SocketAddress::IsMatchingType(from))
    {
        NS_LOG_UNCOND("Received a " << packet->GetSize() << " bytes packet from "
                                    << Inet6SocketAddress::ConvertFrom(from).GetIpv6() << ": "
                                    << buffer);
    }
    // 메모리 정리
    delete[] buffer;
}

// void
// MPQuicKohClient::HandleRecv(Ptr<Socket> socket)
// {
//     NS_LOG_FUNCTION(this << socket);
//
//     Address from;
//     Ptr<Packet> packet = socket->RecvFrom(from);
//     uint8_t* buffer = new uint8_t[packet->GetSize() + 1];
//     packet->CopyData(buffer, packet->GetSize());
//     buffer[packet->GetSize()] = '\0';
//
//     if (InetSocketAddress::IsMatchingType(from))
//     {
//         NS_LOG_UNCOND("Client Received a " << packet->GetSize() << " bytes packet from "
//                                     << InetSocketAddress::ConvertFrom(from).GetIpv4() << ": "
//                                     << buffer);
//     }
//     else if (Inet6SocketAddress::IsMatchingType(from))
//     {
//         NS_LOG_UNCOND("Received a " << packet->GetSize() << " bytes packet from "
//                                     << Inet6SocketAddress::ConvertFrom(from).GetIpv6() << ": "
//                                     << buffer);
//     }
//     // 메모리 정리
//     delete[] buffer;
// }

void
MPQuicKohClient::StopApplication()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendEvent);
    if (m_sendSocket)
    {
        m_sendSocket->Close();
        m_sendSocket = nullptr;
    }
    if (m_Socket)
    {
        m_Socket->Close();
        m_Socket = nullptr;
    }
}



void
MPQuicKohClient::LoadVideoData(std::string filename)
{
    std::ifstream infile(filename);

    if (!infile.is_open())
    {
        NS_LOG_UNCOND("Failed to open trace file: " << filename);
        return;
    }

    uint32_t frameNum;
    std::string typeStr; // 파일에서 텍스트를 읽기 위한 임시 문자열 변수
    uint32_t size;

    uint32_t Iframe_num = 0;
    uint32_t temp_gop = 0;
    bool gop_found=false;

    // 파일 끝까지 읽으면서 벡터에 저장
    // 파일 포맷: [프레임번호] [타입문자열] [사이즈]
    while (infile >> frameNum >> typeStr >> size)
    {
        frameType currentType;

        // 문자열을 Enum으로 변환
        if (typeStr == "IDR")
        {
            currentType = IDR;
            Iframe_num++;
        }
        else if (typeStr == "P")
        {
            currentType = P;
        }
        else if (typeStr == "B")
        {
            currentType = B;
        }
        else
        {
            // 예외 처리: 알 수 없는 타입일 경우 (기본값 설정 또는 로그)
            NS_LOG_UNCOND("Warning: Unknown frame type in trace: " << typeStr);
            NS_FATAL_ERROR("Unknown frame type in trace: " << typeStr);
        }

        temp_gop++;

        if (Iframe_num==2&&!gop_found)
        {
            gop_found = true;
            m_gop=temp_gop-1;
            NS_LOG_UNCOND("gop = " << m_gop);
        }

        // 변환된 enum을 사용하여 구조체 생성 및 저장
        frameData temp{frameNum, currentType, size};
        g_videoFrames.push_back(temp);
    }

    infile.close();
    NS_LOG_UNCOND("Video data loaded. Total frames: " << g_videoFrames.size());
    g_videoFramesIterator = g_videoFrames.begin();
}

void
MPQuicKohClient::ReadVideoData()
{
    // 0.0416 24프레임 전송
    if (g_videoFramesIterator == g_videoFrames.end())
    {
        return;
    }
    frameData temp = *g_videoFramesIterator;
    SendFrame(temp);
    g_videoFramesIterator++;
    m_readVideoDataEvent =
        Simulator::Schedule(m_intervalReadVideoData, &MPQuicKohClient::ReadVideoData, this);
}


void
MPQuicKohClient::SendFrame(frameData fd)
{
    // NS_LOG_UNCOND("MPQuicKohClient::Send");
    NS_LOG_FUNCTION(this);
    static uint16_t streamId = 2;
    Address from;
    Address to;
    m_Socket->GetSockName(from);
    m_Socket->GetPeerName(to);

    Ptr<Packet> p = Create<Packet>(fd.size);

    KohMetadata header;
    header.SetTxTime(Simulator::Now());
    header.SetFrameData(fd);

    p->AddHeader(header);

    m_txTrace(p);
    m_txTraceWithAddresses(p, from, to);

    // NS_LOG_UNCOND("Client KohTag size: " << sizeof(tag));
    // NS_LOG_UNCOND("Client KohTag serial size: " << sizeof(tag.GetSerializedSize()));
    // NS_LOG_UNCOND("Client SeqTsHeader size: " << sizeof(seqTs));
    // NS_LOG_UNCOND("Client SeqTsHeader serial size: " << sizeof(seqTs.GetSerializedSize()));
    // NS_LOG_UNCOND(header.toString());

    if ((m_Socket->Send(p, streamId)) >= 0)
    {
        ++m_sent;
        ++m_totalSent;
        m_totalTx += p->GetSize();
        m_seq += 1;
    }
    streamId = (streamId + 1) % 10 + 1;
}


// void
// MPQuicKohClient::SetTag(KohTag temp)
// {
//     tag = temp;
// }





uint16_t
MPQuicKohClient::GetUeId()//안씀
{
    return tag.ueId;
}

void
MPQuicKohClient::clearCount()
{
    m_sent = 0;
}

void
MPQuicKohClient::SyncSent()
{
    m_sentSync = m_sent;
    m_sent = 0;
}

uint64_t
MPQuicKohClient::GetTotalTx() const // 안씀
{
    return m_totalTx;
}

uint64_t
MPQuicKohClient::GetTotalSent()
{
    return m_totalSent;
}



void
MPQuicKohClient::Send()
{
    // NS_LOG_UNCOND("MPQuicKohClient::Send");
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_sendEvent.IsExpired());
    // NS_LOG_UNCOND("UdpKohClient::Send()");
    Address from;
    Address to;
    m_Socket->GetSockName(from);
    m_Socket->GetPeerName(to);

    Ptr<Packet> p = Create<Packet>(m_size);

    KohMetadata header;
    header.SetTxTime(Simulator::Now());
    // header.SetPacketNum(m_packetCounter++);
    // header.SetPacketSize(p->GetSize()+header.GetSerializedSize());

    p->AddHeader(header);

    m_txTrace(p);
    m_txTraceWithAddresses(p, from, to);

    // NS_LOG_UNCOND("Client KohTag size: " << sizeof(tag));
    // NS_LOG_UNCOND("Client KohTag serial size: " << sizeof(tag.GetSerializedSize()));
    // NS_LOG_UNCOND("Client SeqTsHeader size: " << sizeof(seqTs));
    // NS_LOG_UNCOND("Client SeqTsHeader serial size: " << sizeof(seqTs.GetSerializedSize()));

    if ((m_Socket->Send(p, 1)) >= 0)
    {
        ++m_sent;
        ++m_totalSent;
        m_totalTx += p->GetSize();
        m_seq += 1;
    }
    // NS_LOG_UNCOND("클라이언트에서 보낸 패킷 크기 : "<<p->GetSize());
    if (m_sent < m_count || m_count == 0)
    {
        m_sendEvent = Simulator::Schedule(m_interval, &MPQuicKohClient::Send, this);
    }
}

} // Namespace ns3