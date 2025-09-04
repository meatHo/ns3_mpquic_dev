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
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/uinteger.h"

#include <cstdio>
#include <cstdlib>




namespace ns3 {
    NS_OBJECT_ENSURE_REGISTERED(QuicKohServer); // GetTypeId 위에 추가
    NS_LOG_COMPONENT_DEFINE ("QuicKohServer");


    TypeId
    QuicKohServer::GetTypeId() {
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
        : m_received(0),
          m_lossCounter(0) {
        m_nextClientId = 0;
    }

    void
    QuicKohServer::StartApplication() {
        TypeId tid = TypeId::LookupByName ("ns3::QuicSocketFactory");
        // 소켓 만들어서 대입
        std::cout << "quic koh server StartApplication" << std::endl;

        // if (!m_socket) {
        //     m_socket = Socket::CreateSocket(GetNode(), tid);
        //     InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        //
        //     if (m_socket->Bind(local) == -1) {
        //         NS_FATAL_ERROR("Failed to bind socket");
        //     }
        //     m_socket->Listen();
        //     m_socket->SetRecvCallback(MakeCallback(&QuicKohServer::HandleRead, this));
        // }

        if (!m_socket6) {
            m_socket6 = Socket::CreateSocket(GetNode(), tid);
            Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_port);

            if (m_socket6->Bind(local) == -1) {
                NS_FATAL_ERROR("Failed to bind socket");
            }
            m_socket6->Listen();
            m_socket6->SetRecvCallback(MakeCallback(&QuicKohServer::HandleRead, this));
        }
        NS_LOG_UNCOND("server started");

    }

    void
    QuicKohServer::StopApplication() {
        if (m_socket6) {
            m_socket6->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
        }
        if (m_socket) {
            m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
        }
    }

    uint16_t
    QuicKohServer::GetPacketWindowSize() const {
        return m_lossCounter.GetBitMapSize();
    }

    void
    QuicKohServer::SetPacketWindowSize(uint16_t size) {
        std::cout << "bitmap size:" << size << std::endl;
        m_lossCounter.SetBitMapSize(size);
    }

    uint32_t
    QuicKohServer::GetLost(void) const {
        NS_LOG_FUNCTION(this);
        return m_lossCounter.GetLost();
    }

    uint64_t
QuicKohServer::GetReceived (void) const
    {
        NS_LOG_FUNCTION (this);
        return m_received;
    }


    void
    QuicKohServer::HandleRead(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        Address localAddress;
        while ((packet = socket->RecvFrom(from))) {
            bool clientFound = false;
            uint16_t clientId;
            for (const auto &pair: clients) {
                if (pair.second.address == from) {
                    clientFound = true;
                    clientId = pair.first;
                    from = clients[clientId].address;

                    break;
                }
            }
            // 새 클라이언트 저장
            if (!clientFound) {
                std::cout << "new client detected" << std::endl;
                uint16_t newId = m_nextClientId++;
                clientInfo newClient;
                newClient.address = from;
                newClient.lastSequenceNum = 0;
                newClient.connectionTime = Simulator::Now();
                newClient.packetLossRate = 0;
                newClient.RTT = 0;
                newClient.totalBytesReceived = 0;
                clients[newId] = newClient;
                clientId = newId;
            }

            // 수신
            socket->GetSockName(localAddress);
            m_rxTrace(packet);
            m_rxTraceWithAddresses(packet, from, localAddress);
            if (packet->GetSize() > 0) {
                uint32_t receivedSize = packet->GetSize();
                SeqTsHeader seqTs;
                packet->RemoveHeader(seqTs);
                uint32_t currentSequenceNumber = seqTs.GetSeq();
                if (InetSocketAddress::IsMatchingType(from)) {
                    std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                            << InetSocketAddress::ConvertFrom(from).GetIpv4()
                            << "port: " << InetSocketAddress::ConvertFrom(from).GetPort()
                            << " Sequence Number: " << currentSequenceNumber
                            << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                            << " RXtime: " << Simulator::Now()
                            << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
                } else if (Inet6SocketAddress::IsMatchingType(from)) {
                    std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                            << Inet6SocketAddress::ConvertFrom(from).GetIpv6()
                            << " port: " << Inet6SocketAddress::ConvertFrom(from).GetPort()
                            << " Sequence Number: " << currentSequenceNumber
                            << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                            << " RXtime: " << Simulator::Now()
                            << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
                }

                m_lossCounter.NotifyReceived(currentSequenceNumber);
                m_received++;

                SendPacket(clientId, "good");
                std::cout << "sent to client - clientId : " << clientId << std::endl;
            }
        }
    }

    void
    QuicKohServer::SendPacket(uint16_t clientId, std::string message) {
        auto it = clients.find(clientId);

        if (it == clients.end()) {
            std::cout << "SendPacket failed: Client with ID " << clientId << " not found." << std::endl;
            return;
        }

        Address destAddress = it->second.address;
        Ptr<Packet> packet = Create<Packet>((uint8_t *) message.c_str(), message.length());

        // if (InetSocketAddress::IsMatchingType(destAddress))
        // {
        //     // IPv4 주소일 경우 m_socket 사용
        //     m_socket->SendTo(packet, 0, destAddress);
        //     NS_LOG_INFO("Sent an IPv4 packet to client ID " << clientId);
        // }
        if (Inet6SocketAddress::IsMatchingType(destAddress)) {
            // IPv6 주소일 경우 m_socket6 사용
            m_socket6->SendTo(packet, 0, destAddress);
            std::cout << "Sent an IPv6 packet to client ID " << clientId << std::endl;;
        }
    }
} // Namespace ns3
