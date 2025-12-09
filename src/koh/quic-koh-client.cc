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
#include "quic-koh-client.h"

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

NS_LOG_COMPONENT_DEFINE("QuicKohClient");

NS_OBJECT_ENSURE_REGISTERED(QuicKohClient);

TypeId
QuicKohClient::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::QuicKohClient")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<QuicKohClient>()
            .AddAttribute(
                "MaxPackets",
                "The maximum number of packets the application will send (zero means infinite)",
                UintegerValue(100),
                MakeUintegerAccessor(&QuicKohClient::m_count),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("Interval",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&QuicKohClient::m_interval),
                          MakeTimeChecker())
            .AddAttribute("IntervalSl",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&QuicKohClient::m_intervalSl),
                          MakeTimeChecker())
            .AddAttribute("IntervalReadVideoData",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&QuicKohClient::m_intervalReadVideoData),
                          MakeTimeChecker())
            .AddAttribute("IntervalUu",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&QuicKohClient::m_intervalUu),
                          MakeTimeChecker())
            .AddAttribute("slServerAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&QuicKohClient::m_slServerAddress),
                          MakeAddressChecker())
            .AddAttribute("slServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&QuicKohClient::m_slServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("uuServerAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&QuicKohClient::m_uuServerAddress),
                          MakeAddressChecker())
            .AddAttribute("uuServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&QuicKohClient::m_uuServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("PacketSize",
                          "Size of packets generated. The minimum packet size is 12 bytes which is "
                          "the size of the header carrying the sequence number and the time stamp.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&QuicKohClient::m_size),
                          MakeUintegerChecker<uint32_t>(12, 65507))
            .AddTraceSource("Tx",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&QuicKohClient::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("TxWithAddresses",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&QuicKohClient::m_txTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

QuicKohClient::QuicKohClient()
{
    NS_LOG_FUNCTION(this);
    m_sentSl = 0;
    m_sentUu = 0;
    m_totalTx = 0;
    m_uuSocket = nullptr;
    m_slSocket = nullptr;
    m_sendSocket = nullptr;
    m_sendUuEvent = EventId();
    m_sendSlEvent = EventId();
    m_lastUsedStream = 1;
    m_slSocketConnected = false;
    m_seqSl = 1;
    m_seqUu = 0;
    m_packetCounter = 0;

    m_totalSent = 0;
}

QuicKohClient::~QuicKohClient()
{
    NS_LOG_FUNCTION(this);
}

void
QuicKohClient::StartApplication()
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

    // Uu 소켓 설정
    m_uuSocket = Socket::CreateSocket(GetNode(), TypeId(QuicSocketFactory::GetTypeId()));
    m_uuSocket->Bind();
    m_uuSocket->BindToNetDevice(m_devUu); // Connect 전에 Bind
    if (Ipv6Address::IsMatchingType(m_uuServerAddress))
    {
        NS_LOG_UNCOND("uu ipv6 socket connection started");
        m_uuSocket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_uuServerAddress), m_uuServerPort));
    }
    else
    {
        NS_LOG_UNCOND("uu ipv4 socket connection started");
        m_uuSocket->Connect(
            InetSocketAddress(Ipv4Address::ConvertFrom(m_uuServerAddress), m_uuServerPort));
    }

    // Sl 소켓 설정
    m_slSocket = Socket::CreateSocket(GetNode(), TypeId(QuicSocketFactory::GetTypeId()));
    m_slSocket->Bind();
    m_slSocket->BindToNetDevice(m_devSl); // Connect 전에 Bind
    if (Ipv6Address::IsMatchingType(m_slServerAddress))
    {
        NS_LOG_UNCOND("sl socket connection started");
        m_slSocket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_slServerAddress), m_slServerPort));
    }
    else
    {
        NS_LOG_UNCOND("sl ipv4 socket connection started");
        m_slSocket->Connect(
            InetSocketAddress(Ipv4Address::ConvertFrom(m_slServerAddress), m_slServerPort));
    }

    // 수신 콜백 설정 (양방향 통신 시 필요)
    m_slSocket->SetRecvCallback(MakeCallback(&QuicKohClient::HandleRecv, this));
    m_uuSocket->SetRecvCallback(MakeCallback(&QuicKohClient::HandleRecv, this));

    // 초기 인터페이스는 Uu로 설정
    // m_sendSocket = m_uuSocket;
    // NS_LOG_UNCOND("Client starts with uu interface.");

    m_readVideoDataEvent = Simulator::Schedule(Seconds(0.0), &QuicKohClient::ReadVideoData, this);

    // m_sendSlEvent = Simulator::Schedule(Seconds(0.0), &QuicKohClient::SendSl, this);
    // m_sendUuEvent = Simulator::Schedule(Seconds(0.0), &QuicKohClient::SendUu, this);

    // Simulator::Schedule(Seconds(0.0), &QuicKohClient::SyncSent, this);
}

void
QuicKohClient::HandleRecv(Ptr<Socket> socket)
{
    SyncSent();

    NS_LOG_UNCOND("QuicKohClient::HandleRecv");
    NS_LOG_FUNCTION(this << socket);
    Address from;
    Ptr<Packet> packet = socket->RecvFrom(from);
    uint8_t* buffer = new uint8_t[packet->GetSize() + 1];
    packet->CopyData(buffer, packet->GetSize());
    buffer[packet->GetSize()] = '\0';

    if (InetSocketAddress::IsMatchingType(from))
    {
        NS_LOG_UNCOND("Received a " << (socket == m_slSocket ? tagSl.ueId : tagUu.ueId) << " ue "
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
// QuicKohClient::HandleRecv(Ptr<Socket> socket)
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
QuicKohClient::StopApplication()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendSlEvent);
    Simulator::Cancel(m_sendUuEvent);
    if (m_sendSocket)
    {
        m_sendSocket->Close();
        m_sendSocket = nullptr;
    }
    if (m_uuSocket)
    {
        m_uuSocket->Close();
        m_uuSocket = nullptr;
    }
    if (m_slSocket)
    {
        m_slSocket->Close();
        m_slSocket = nullptr;
    }
}

void
QuicKohClient::SendSplit()
{
}

void
QuicKohClient::LoadVideoData(std::string filename)
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
QuicKohClient::ReadVideoData()
{
    // 0.0416 24프레임 전송
    if (g_videoFramesIterator == g_videoFrames.end())
    {
        return;
    }
    frameData temp = *g_videoFramesIterator;
    SelectInterface(temp.type);
    switch (temp.type)
    {
    case IDR:
        SendUuFrame(temp);
        break;
    default:
        SendSlFrame(temp);
        break;
    }
    g_videoFramesIterator++;
    m_readVideoDataEvent =
        Simulator::Schedule(m_intervalReadVideoData, &QuicKohClient::ReadVideoData, this);
}

InterfaceType
QuicKohClient::SelectInterface(frameType)
{
    Ptr<QuicSocketBase> quicSlSocket = DynamicCast<QuicSocketBase>(m_slSocket);
    Time smoothedRttSl = quicSlSocket->GetSmoothedRtt();
    Time rttVarSl = quicSlSocket->GetRttVar();
    Time minRttSl = quicSlSocket->GetMinRtt();

    uint32_t maxTxBufSL = quicSlSocket->GetSocketSndBufSize();
    uint32_t availableTxSl = quicSlSocket->GetTxAvailable();

    Ptr<QuicSocketBase> quicUuSocket = DynamicCast<QuicSocketBase>(m_uuSocket);
    Time smoothedRttUu = quicUuSocket->GetSmoothedRtt();
    Time rttVarUu = quicUuSocket->GetRttVar();
    Time minRttUu = quicUuSocket->GetMinRtt();

    uint32_t maxTxBufUu = quicUuSocket->GetSocketSndBufSize();
    uint32_t availableTxUu = quicUuSocket->GetTxAvailable();

    NS_LOG_UNCOND("smoothedRttSl: "<<smoothedRttSl<<", rttVarSl: "<<rttVarSl<<", minRttSl: "<<minRttSl<<", maxTxBufSl: "<<maxTxBufSL<<", availableTxSl: "<<availableTxSl);
    NS_LOG_UNCOND("smoothedRttUu: "<<smoothedRttUu<<", rttVarUu: "<<rttVarUu<<", minRttUu: "<<minRttUu<<", maxTxBufUu: "<<maxTxBufUu<<", availableTxUu: "<<availableTxUu);

    NS_LOG_DEBUG("smoothedRttSl: "<<smoothedRttSl<<", rttVarSl: "<<rttVarSl<<", minRttSl: "<<minRttSl<<", maxTxBufSl: "<<maxTxBufSL<<", availableTxSl: "<<availableTxSl);
    NS_LOG_DEBUG("smoothedRttUu: "<<smoothedRttUu<<", rttVarUu: "<<rttVarUu<<", minRttUu: "<<minRttUu<<", maxTxBufUu: "<<maxTxBufUu<<", availableTxUu: "<<availableTxUu);

    return UU;
}

void
QuicKohClient::SendSlFrame(frameData fd)
{
    // NS_LOG_UNCOND("QuicKohClient::Send");
    NS_LOG_FUNCTION(this);
    // NS_LOG_UNCOND("UdpKohClient::Send()");
    static uint16_t streamId = 2;
    Address from;
    Address to;
    m_slSocket->GetSockName(from);
    m_slSocket->GetPeerName(to);

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

    if ((m_slSocket->Send(p, streamId)) >= 0)
    {
        ++m_sentSl;
        ++m_totalSent;
        m_totalTx += p->GetSize();
        m_seqSl += 1;
    }
    streamId = (streamId + 1) % 10 + 1;
}

void
QuicKohClient::SendUuFrame(frameData fd)
{
    // NS_LOG_UNCOND("QuicKohClient::Send");
    NS_LOG_FUNCTION(this);
    // NS_LOG_UNCOND("UdpKohClient::Send()");
    static uint16_t streamId = 2;
    Address from;
    Address to;
    m_uuSocket->GetSockName(from);
    m_uuSocket->GetPeerName(to);

    Ptr<Packet> p = Create<Packet>(fd.size);

    KohMetadata header;
    header.SetTxTime(Simulator::Now());
    header.SetFrameData(fd);

    p->AddHeader(header);

    m_txTrace(p);
    m_txTraceWithAddresses(p, from, to);    // NS_LOG_UNCOND("Client KohTag size: " << sizeof(tag));


    // NS_LOG_UNCOND("Client KohTag serial size: " << sizeof(tag.GetSerializedSize()));
    // NS_LOG_UNCOND("Client SeqTsHeader size: " << sizeof(seqTs));
    // NS_LOG_UNCOND("Client SeqTsHeader serial size: " << sizeof(seqTs.GetSerializedSize()));
    // NS_LOG_UNCOND("SendUuFrame "<<header.toString());

    if ((m_uuSocket->Send(p, streamId)) >= 0)
    {
        ++m_sentUu;
        ++m_totalSent;
        m_totalTx += p->GetSize();
        m_seqUu += 1;
    }
    streamId = (streamId + 1) % 10 + 1;
    // m_streamId += 4;
}


// void
// QuicKohClient::SetTag(KohTag temp)
// {
//     tag = temp;
// }

// [핵심 수정] changeInterface() 함수
void
QuicKohClient::changeInterface()
{
    // Ipv6StaticRoutingHelper ipv6RoutingHelper;
    // Ptr<Ipv6> ipv6 = GetNode()->GetObject<Ipv6>();
    // Ptr<Ipv6StaticRouting> staticRouting = ipv6RoutingHelper.GetStaticRouting(ipv6);

    if (m_sendSocket == m_uuSocket)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s: ---> Switching client interface to SL socket <---");

        // // 1. 라우팅 규칙 추가: SL 서버 주소로 가려면 RSU를 거쳐가도록 설정
        // uint32_t slInterfaceIndex = ipv6->GetInterfaceForDevice(m_devSl);
        // staticRouting->AddHostRouteTo(Ipv6Address::ConvertFrom(m_slServerAddress),
        //                               Ipv6Address::ConvertFrom(m_slNextHopIp),
        //                               slInterfaceIndex);
        // NS_LOG_UNCOND("Route ADDED: Dst=" << m_slServerAddress << " via " << m_slNextHopIp);

        // 2. 전송 소켓을 SL 소켓으로 변경

        if (!m_slSocketConnected)
        {
            NS_LOG_UNCOND("First time using SL socket. Binding and Connecting...");
            m_slSocket->Bind();
            m_slSocket->BindToNetDevice(m_devSl);
            if (Ipv6Address::IsMatchingType(m_slServerAddress))
            {
                m_slSocket->Connect(Inet6SocketAddress(Ipv6Address::ConvertFrom(m_slServerAddress),
                                                       m_slServerPort));
            }
            else
            {
                m_slSocket->Connect(
                    InetSocketAddress(Ipv4Address::ConvertFrom(m_slServerAddress), m_slServerPort));
            }
            m_slSocketConnected = true; // 연결되었음을 표시
        }
        m_sendSocket = m_slSocket;
    }
    else
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s: ---> Switching client interface to UU socket <---");

        // [수정] 라우팅 규칙을 '인덱스'로 삭제하는 로직
        // 1. 추가했던 SL 경로 라우팅 규칙을 찾아서 삭제
        // int32_t routeIndex = -1;
        // for (uint32_t i = 0; i < staticRouting->GetNRoutes(); i++)
        // {
        //     Ipv6RoutingTableEntry route = staticRouting->GetRoute(i);
        //     // 우리가 추가했던 경로와 목적지 주소, Next Hop 주소가 같은지 확인
        //     if (route.GetDest() == Ipv6Address::ConvertFrom(m_slServerAddress) &&
        //         route.GetGateway() == Ipv6Address::ConvertFrom(m_slNextHopIp))
        //     {
        //         routeIndex = i;
        //         break;
        //     }
        // }

        // if (routeIndex != -1)
        // {
        //     staticRouting->RemoveRoute(routeIndex);
        //     NS_LOG_UNCOND("Route REMOVED: Dst=" << m_slServerAddress);
        // }
        // else
        // {
        //     NS_LOG_WARN("Could not find route to remove for " << m_slServerAddress);
        // }

        // 2. 전송 소켓을 Uu 소켓으로 다시 변경 (Default Route를 따름)
        m_sendSocket = m_uuSocket;

        // NS_LOG_UNCOND("sl ipv4 socket connection started");
        // m_uuSocket->Connect(
        //     InetSocketAddress(Ipv4Address::ConvertFrom(m_uuServerAddress), m_uuServerPort));
    }
}

void
QuicKohClient::setInterface(Ptr<NetDevice> uu, Ptr<NetDevice> sl) // 안씀
{
    m_devSl = sl;
    m_devUu = uu;
}

// uint16_t
// QuicKohClient::GetUeId()//안씀
// {
//     return tag.ueId;
// }

void
QuicKohClient::clearCount()
{
    m_sentSl = 0;
    m_sentUu = 0;
}

void
QuicKohClient::SyncSent()
{
    m_sentSlSync = m_sentSl;
    m_sentUuSync = m_sentUu;
    m_sentSl = 0;
    m_sentUu = 0;
}

uint64_t
QuicKohClient::GetTotalTx() const // 안씀
{
    return m_totalTx;
}

uint64_t
QuicKohClient::GetTotalSent()
{
    return m_totalSent;
}

void
QuicKohClient::SendSl()
{
    // NS_LOG_UNCOND("QuicKohClient::Send");
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_sendSlEvent.IsExpired());
    // NS_LOG_UNCOND("UdpKohClient::Send()");
    Address from;
    Address to;
    m_slSocket->GetSockName(from);
    m_slSocket->GetPeerName(to);

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

    if ((m_slSocket->Send(p, 1)) >= 0)
    {
        ++m_sentSl;
        ++m_totalSent;
        m_totalTx += p->GetSize();
        m_seqSl += 1;
    }
    // NS_LOG_UNCOND("클라이언트에서 보낸 패킷 크기 : "<<p->GetSize());
    if (m_sentSl < m_count || m_count == 0)
    {
        m_sendSlEvent = Simulator::Schedule(m_intervalSl, &QuicKohClient::SendSl, this);
    }
}

void
QuicKohClient::SendUu()
{
    // NS_LOG_UNCOND("QuicKohClient::Send");
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_sendUuEvent.IsExpired());
    // NS_LOG_UNCOND("UdpKohClient::Send()");
    Address from;
    Address to;
    m_uuSocket->GetSockName(from);
    m_uuSocket->GetPeerName(to);

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

    if ((m_uuSocket->Send(p, 1)) >= 0)
    {
        ++m_sentUu;
        ++m_totalSent;
        m_totalTx += p->GetSize();
        m_seqUu += 2;
    }
    // NS_LOG_UNCOND("클라이언트에서 보낸 패킷 크기 : "<<p->GetSize());
    if (m_sentUu < m_count || m_count == 0)
    {
        m_sendUuEvent = Simulator::Schedule(m_intervalUu, &QuicKohClient::SendUu, this);
    }
}

} // Namespace ns3