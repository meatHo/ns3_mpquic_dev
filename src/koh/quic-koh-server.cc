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
#include "quic-koh-server.h"

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

namespace ns3
{
NS_OBJECT_ENSURE_REGISTERED(QuicKohServer); // GetTypeId 위에 추가
NS_LOG_COMPONENT_DEFINE("QuicKohServer");

TypeId
QuicKohServer::GetTypeId()
{
    static TypeId tid =
        TypeId("QuicKohServer")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<QuicKohServer>()
            // 포트는 Attribute로 설정하는 것이 표준입니다.
            .AddAttribute("Port",
                          "Port on which we listen for incoming packets.",
                          UintegerValue(9000), // 기본 포트값
                          MakeUintegerAccessor(&QuicKohServer::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("PacketWindowSize",
                          "The size of the window used to compute the packet loss. This value "
                          "should be a multiple of 8.",
                          UintegerValue(128),
                          MakeUintegerAccessor(&QuicKohServer::GetPacketWindowSize,
                                               &QuicKohServer::SetPacketWindowSize),
                          MakeUintegerChecker<uint16_t>(8, 256))
            .AddTraceSource("Rx",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&QuicKohServer::m_rxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("RxWithAddresses",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&QuicKohServer::m_rxTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

QuicKohServer::QuicKohServer()
    : m_totalReceived(0),
      m_lossCounter(0)
{
    m_nextClientId = 0;
    m_totalRx=0;
    m_recvPerUe.clear();
    m_latencySumPerUe.clear();
    m_latencyCountPerUe.clear();
}

void
QuicKohServer::StartApplication()
{
    // 소켓 만들어서 대입
    std::cout << "quic koh server StartApplication" << std::endl;

    // m_socket6 = Socket::CreateSocket(GetNode(), TypeId(QuicSocketFactory::GetTypeId()));
    // Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_port);
    //
    // if (m_socket6->Bind(local) == -1)
    // {
    //     NS_FATAL_ERROR("Failed to bind socket");
    // }
    // m_socket6->Listen();
    // m_socket6->SetRecvCallback (MakeCallback (&QuicKohServer::HandleRead, this));

    // 서버 초기화
    m_socket = Socket::CreateSocket(GetNode(), TypeId(QuicSocketFactory::GetTypeId()));
    InetSocketAddress local_ipv4 = InetSocketAddress(Ipv4Address::GetAny(), m_port);

    if (m_socket->Bind(local_ipv4) == -1)
    {
        NS_FATAL_ERROR("Failed to bind socket");
    }

    m_socket->Listen();

    // 새로운 연결이 들어오면 HandleAccept 호출
    m_socket->SetRecvCallback(MakeCallback(&QuicKohServer::HandleRead, this));

    NS_LOG_UNCOND("QUIC server started on port " << m_port);
    // Simulator::Schedule(Seconds(3.0), &QuicKohServer::SendUeStats, this, 0);
}

// HandleRead 이전에 이 함수를 추가합니다.
void
QuicKohServer::HandleAccept(Ptr<Socket> newSocket, const Address& from)
{
    NS_LOG_UNCOND("New QUIC connection accepted from: ");

}

void
QuicKohServer::StopApplication()
{
    if (m_socket6)
    {
        m_socket6->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
    if (m_socket)
    {
        m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

uint16_t
QuicKohServer::GetPacketWindowSize() const
{
    return m_lossCounter.GetBitMapSize();
}

void
QuicKohServer::SetPacketWindowSize(uint16_t size)
{
    std::cout << "bitmap size:" << size << std::endl;
    m_lossCounter.SetBitMapSize(size);
}

uint32_t
QuicKohServer::GetLost(void) const
{
    NS_LOG_FUNCTION(this);
    return m_lossCounter.GetLost();
}

uint32_t
QuicKohServer::GetRecvCount(uint16_t ueId)
{
    auto it = m_recvPerUe.find(ueId);
    uint32_t temp = 0;
    if (it != m_recvPerUe.end())
    {
        temp = it->second;
        it->second = 0;
        return temp;
    }
    return 0;
}

double
QuicKohServer::GetLatency(uint16_t ueId)
{
    auto itSum = m_latencySumPerUe.find(ueId);
    auto itCnt = m_latencyCountPerUe.find(ueId);

    if (itSum != m_latencySumPerUe.end() && itCnt != m_latencyCountPerUe.end() && itCnt->second > 0)
    {
        double avg = itSum->second / itCnt->second;

        itSum->second = 0.0;
        itCnt->second = 0;

        return avg;
    }
    return 0.0;
}

void
QuicKohServer::clearCount(uint16_t ueId)
{
    m_recvPerUe[ueId] = 0;
}

uint64_t
QuicKohServer::GetTotalRecv()
{
    return m_totalReceived;
}

void
QuicKohServer::HandleRead(Ptr<Socket> socket)
{
    // NS_LOG_UNCOND("QuicKohServer::HandleRead");
    Ptr<Packet> packet;
    Address from;
    Address localAddress;
    // double latency;
    uint16_t ueId;

    while ((packet = socket->RecvFrom(from)))
    {
        bool clientFound = false;
        uint16_t clientId;
        for (const auto& pair : clients)
        {
            if (pair.second.address == from)
            {
                clientFound = true;
                clientId = pair.first;
                from = clients[clientId].address;

                break;
            }
        }
        InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
        // NS_LOG_UNCOND("who's ip : handle read: " << addr.GetIpv4() << " Port: " << addr.GetPort());
        // 새 클라이언트 저장
        if (!clientFound)
        {
            NS_LOG_UNCOND("new client detected");
            uint16_t newId = m_nextClientId++;
            clientInfos newClient;
            newClient.address = from;
            newClient.lastSequenceNum = 0;
            newClient.connectionTime = Simulator::Now();
            newClient.packetLossRate = 0;
            newClient.RTT = 0;
            newClient.socket=socket;
            newClient.totalBytesReceived = 0;
            clients[newId] = newClient;
            clientId = newId;
            // std::cout << "New client detected | Address: " << newClient.address
            //      << " | Connection Time: " << newClient.connectionTime << std::endl;
        }

        // 수신
        socket->GetSockName(localAddress);
        m_rxTrace(packet);
        m_rxTraceWithAddresses(packet, from, localAddress);

        KohTag tag;
        if (packet->RemovePacketTag(tag))
        {
            Time tx = tag.txTime;
            ueId = tag.ueId;
            double frac = fmod(Simulator::Now().GetSeconds(), 1.0);

            // 매초 0.1초 ~ 1.0초 카운트 증가
            if (frac >= 0.1 && frac < 1.0)
            {
                m_recvPerUe[ueId]++;
            }

            // m_recvPerUe[ueId]++;

            // latency X
            double latency;
            latency = (Simulator::Now() - tx).GetSeconds();
            // NS_LOG_UNCOND("UE=" << ueId << " delay=" << latency << " s");
            m_latencySumPerUe[ueId] += latency;
            m_latencyCountPerUe[ueId] += 1;
        }

        if (packet->GetSize() > 12)
        {
            uint32_t receivedSize = packet->GetSize();
            m_totalRx += receivedSize;
            NS_LOG_UNCOND("total rx : "<<m_totalRx);
            // NS_LOG_UNCOND("server application received "<<receivedSize<<"byte");
            SeqTsHeader seqTs;
            // NS_LOG_UNCOND("서버에서 받은 패킷 크기 : "<<packet->GetSize()<<" seqt사이즈:"<<sizeof(seqTs)<<"  serialize :"<<seqTs.GetSerializedSize());
            packet->RemoveHeader(seqTs);
            NS_LOG_UNCOND("ip :" << addr.GetIpv4() <<"  size :"<<packet->GetSize()<<"  seq :"<<seqTs.GetSeq());

            uint32_t currentSequenceNumber = seqTs.GetSeq();
            if (InetSocketAddress::IsMatchingType(from))
            {
                // std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                //           << InetSocketAddress::ConvertFrom(from).GetIpv4()
                //           << " port: " << InetSocketAddress::ConvertFrom(from).GetPort()
                //           << " Sequence Number: " << currentSequenceNumber
                //           << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                //           << " RXtime: " << Simulator::Now()
                //           << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
            }
            else if (Inet6SocketAddress::IsMatchingType(from))
            {
                std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                          << Inet6SocketAddress::ConvertFrom(from).GetIpv6()
                          << " port: " << Inet6SocketAddress::ConvertFrom(from).GetPort()
                          << " Sequence Number: " << currentSequenceNumber
                          << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                          << " RXtime: " << Simulator::Now()
                          << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
            }

            m_lossCounter.NotifyReceived(currentSequenceNumber);
            m_totalReceived++;

            // SendPacket(socket, from, std::string("good"));
            // NS_LOG_UNCOND("handleread socket : "<<socket);
            // std::cout << "sent to client - clientId : " << clientId << "  Address :
            // "<<InetSocketAddress::ConvertFrom(clients[clientId].address).GetIpv4()<< std::endl;
        }
    }
}

void QuicKohServer::SendPacket(Ptr<Socket> socket, Address from, const std::string &message)
{
    NS_LOG_UNCOND("QuicKohServer::SendPacket");

    if (InetSocketAddress::IsMatchingType(from))
    {
        InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
        NS_LOG_UNCOND("Sending stats to Client IP: " << addr.GetIpv4() << " Port: " << addr.GetPort());
    }
    Ptr<Packet> packet = Create<Packet>((uint8_t*) message.c_str(), message.length());
    socket->SendTo(packet,0,from);
}

// void
// QuicKohServer::SendPacket(uint16_t clientId, std::string message)
// {
//     auto it = clients.find(clientId);
//
//     if (it == clients.end())
//     {
//         std::cout << "SendPacket failed: Client with ID " << clientId << " not found." << std::endl;
//         return;
//     }
//
//     Address destAddress = it->second.address;
//     Ptr<Packet> packet = Create<Packet>((uint8_t*)message.c_str(), message.length());
//
//     // if (InetSocketAddress::IsMatchingType(destAddress))
//     // {
//     //     // IPv4 주소일 경우 m_socket 사용
//     //     m_socket->SendTo(packet, 0, destAddress);
//     //     NS_LOG_INFO("Sent an IPv4 packet to client ID " << clientId);
//     // }
//     if (Inet6SocketAddress::IsMatchingType(destAddress))
//     {
//         // IPv6 주소일 경우 m_socket6 사용
//         m_socket6->SendTo(packet, 0, destAddress);
//         std::cout << "Sent an IPv6 packet to client ID " << clientId << std::endl;
//     }
//     else
//     {
//         m_socket->SendTo(packet, 0, destAddress);
//         // std::cout << "Sent an IPv4 packet to client ID " << clientId << std::endl;
//     }
//     Simulator::Schedule(Seconds(1.0), &QuicKohServer::SendPacket, this, "fuckkjhgkjhgkjhgkjhg");
// }

void
QuicKohServer::Send()
{
    // NS_LOG_UNCOND("QuicKohServer::Send");
    Ipv4Address ipv4Dest = Ipv4Address("7.0.0.2");
    Address destAddress = Address(ipv4Dest);
    std::string message = "씨발";
    Ptr<Packet> packet = Create<Packet>((uint8_t*)message.c_str(), message.length());

    // if (InetSocketAddress::IsMatchingType(destAddress))
    // {
    //     // IPv4 주소일 경우 m_socket 사용
    //     m_socket->SendTo(packet, 0, destAddress);
    //     NS_LOG_INFO("Sent an IPv4 packet to client ID " << clientId);
    // }
    if (Inet6SocketAddress::IsMatchingType(destAddress))
    {
        // IPv6 주소일 경우 m_socket6 사용
        m_socket6->SendTo(packet, 0, destAddress);
        std::cout << "Sent an IPv6 packet to client ID " << std::endl;
    }
    else
    {
        m_socket->SendTo(packet, 0, destAddress);
        std::cout << "Sent an IPv4 packet to client ID " << std::endl;
    }
    Simulator::Schedule(Seconds(1.0), &QuicKohServer::Send, this);
}

void
QuicKohServer::SendUeStats(uint16_t ueId)
{
    // NS_LOG_UNCOND("QuicKohServer::SendUeStats");
    auto it = clients.find(ueId);

    if (it == clients.end())
    {
        std::cout << "SendPacket failed: Client with ID " << ueId << " not found." << std::endl;
        return;
    }

    Address destAddress = it->second.address;
    KStats stats;
    stats.ueId = ueId;
    stats.recvCount = GetRecvCount(ueId);
    stats.avgLatency = GetLatency(ueId);

    Ptr<Socket> socket = it->second.socket;
    // NS_LOG_UNCOND("SendUeStats socket : "<<socket);
    if (InetSocketAddress::IsMatchingType(destAddress))
    {
        InetSocketAddress addr = InetSocketAddress::ConvertFrom(destAddress);
        NS_LOG_UNCOND("Sending stats to Client IP: " << addr.GetIpv4() << " Port: " << addr.GetPort());
    }

    Ptr<Packet> packet = Create<Packet>((uint8_t*)&stats, sizeof(KStats));
    // socket->Send(packet);
    socket->SendTo(packet,0,destAddress);
    // NS_LOG_UNCOND("server sent packet to client "<<Simulator::Now().GetSeconds());
    if (Simulator::Now().GetSeconds() <= 58)
    {
        Simulator::Schedule(Seconds(1.0), &QuicKohServer::SendUeStats, this, 0);
    }
}

} // namespace ns3
  // Namespace ns3