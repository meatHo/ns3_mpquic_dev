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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Amine Ismail <amine.ismail@sophia.inria.fr>
 *                      <amine.ismail@udcast.com>
 */
#include "udp-koh-client.h"

#include "./../applications/model/seq-ts-header.h"

#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"

#include <cstdio>
#include <cstdlib>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("UdpKohClient");

NS_OBJECT_ENSURE_REGISTERED(UdpKohClient);

TypeId
UdpKohClient::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UdpKohClient")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<UdpKohClient>()
            .AddAttribute(
                "MaxPackets",
                "The maximum number of packets the application will send (zero means infinite)",
                UintegerValue(100),
                MakeUintegerAccessor(&UdpKohClient::m_count),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("Interval",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&UdpKohClient::m_interval),
                          MakeTimeChecker())
            .AddAttribute("slServerAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&UdpKohClient::m_slServerAddress),
                          MakeAddressChecker())
            .AddAttribute("slServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&UdpKohClient::m_slServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("uuServerAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&UdpKohClient::m_uuServerAddress),
                          MakeAddressChecker())
            .AddAttribute("uuServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&UdpKohClient::m_uuServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("recvPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&UdpKohClient::m_recvPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Tos",
                          "The Type of Service used to send IPv4 packets. "
                          "All 8 bits of the TOS byte are set (including ECN bits).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&UdpKohClient::m_tos),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("PacketSize",
                          "Size of packets generated. The minimum packet size is 12 bytes which is "
                          "the size of the header carrying the sequence number and the time stamp.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&UdpKohClient::m_size),
                          MakeUintegerChecker<uint32_t>(12, 65507))
            .AddTraceSource("Tx",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&UdpKohClient::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("TxWithAddresses",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&UdpKohClient::m_txTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

UdpKohClient::UdpKohClient()
{
    NS_LOG_FUNCTION(this);
    m_sent = 0;
    m_totalTx = 0;
    m_uuSocket = nullptr;
    m_slSocket = nullptr;
    m_sendEvent = EventId();
}

UdpKohClient::~UdpKohClient()
{
    NS_LOG_FUNCTION(this);
}

void
UdpKohClient::StartApplication()
{
    NS_LOG_FUNCTION(this);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    // listening 소켓 설정
    m_recvSocket = Socket::CreateSocket(GetNode(), tid);
    Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_recvPort);
    if (m_recvSocket->Bind(local) == -1)
    {
        NS_FATAL_ERROR("UdpRelay: Failed to bind In-Socket");
    }// todo: handle read 안만들음 나중에 콜백으로 이으셈 필요하면

    // Uu 소켓 설정
    m_uuSocket = Socket::CreateSocket(GetNode(), tid);
    m_uuSocket->BindToNetDevice(m_devUu); // Connect 전에 Bind
    if (Ipv6Address::IsMatchingType(m_uuServerAddress))
    {
        NS_LOG_UNCOND("uu socket connection started");
        m_uuSocket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_uuServerAddress), m_uuServerPort));
    }

    // Sl 소켓 설정
    m_slSocket = Socket::CreateSocket(GetNode(), tid);
    m_slSocket->BindToNetDevice(m_devSl); // Connect 전에 Bind
    if (Ipv6Address::IsMatchingType(m_slServerAddress))
    {
        NS_LOG_UNCOND("sl socket connection started");
        m_slSocket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_slServerAddress), m_slServerPort));
    }

    // 수신 콜백 설정 (양방향 통신 시 필요)
    m_uuSocket->SetRecvCallback(MakeCallback(&UdpKohClient::HandleRecv, this));
    m_slSocket->SetRecvCallback(MakeCallback(&UdpKohClient::HandleRecv, this));

    // 초기 인터페이스는 Uu로 설정
    m_sendSocket = m_uuSocket;
    NS_LOG_UNCOND("Client starts with uu interface.");

    m_sendEvent = Simulator::Schedule(Seconds(0.0), &UdpKohClient::Send, this);
}

void
UdpKohClient::HandleRecv(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    Address from;
    Ptr<Packet> packet = socket->RecvFrom(from);
    uint8_t* buffer = new uint8_t[packet->GetSize() + 1];
    packet->CopyData(buffer, packet->GetSize());
    buffer[packet->GetSize()] = '\0';

    if (InetSocketAddress::IsMatchingType(from))
    {
        NS_LOG_UNCOND("Received a " << packet->GetSize() << " bytes packet from "
                                    << InetSocketAddress::ConvertFrom(from).GetIpv4() << ": "
                                    << buffer);
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

void
UdpKohClient::StopApplication()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendEvent);
}

void
UdpKohClient::Send()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(m_sendEvent.IsExpired());
    NS_LOG_UNCOND("UdpKohClient::Send()");
    Address from;
    Address to;
    m_sendSocket->GetSockName(from);
    m_sendSocket->GetPeerName(to);
    SeqTsHeader seqTs;
    seqTs.SetSeq(m_sent);
    NS_ABORT_IF(m_size < seqTs.GetSerializedSize());
    Ptr<Packet> p = Create<Packet>(m_size - seqTs.GetSerializedSize());

    // Trace before adding header, for consistency with PacketSink
    m_txTrace(p);
    m_txTraceWithAddresses(p, from, to);

    p->AddHeader(seqTs);
    if ((m_sendSocket->Send(p)) >= 0)
    {
        ++m_sent;
        m_totalTx += p->GetSize();
#ifdef NS3_LOG_ENABLE
        NS_LOG_INFO("TraceDelay TX " << m_size << " bytes to " << m_peerAddressString << " Uid: "
                                     << p->GetUid() << " Time: " << (Simulator::Now()).As(Time::S));
#endif // NS3_LOG_ENABLE
    }
#ifdef NS3_LOG_ENABLE
    else
    {
        NS_LOG_INFO("Error while sending " << m_size << " bytes to " << m_peerAddressString);
    }
#endif // NS3_LOG_ENABLE

    if (m_sent < m_count || m_count == 0)
    {
        m_sendEvent = Simulator::Schedule(m_interval, &UdpKohClient::Send, this);
    }
}

uint64_t
UdpKohClient::GetTotalTx() const
{
    return m_totalTx;
}

void
UdpKohClient::SelectInterface(Ptr<Socket> socket)
{
    // 여기에 rsu가 좋은지 gnb가 좋은지 고르는 함수 추가
    m_sendSocket = socket;
}

// [핵심 수정] changeInterface() 함수
void
UdpKohClient::changeInterface()
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
    }
}

void
UdpKohClient::setInterface(Ptr<NetDevice> uu, Ptr<NetDevice> sl)
{
    m_devSl = sl;
    m_devUu = uu;
}

} // Namespace ns3