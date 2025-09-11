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
#include "ns3/quic-socket-factory.h"
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
        m_socket->SetRecvCallback (MakeCallback (&QuicKohServer::HandleRead, this));

        NS_LOG_UNCOND("QUIC server started on port " << m_port);

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
QuicKohServer::HandleRead(Ptr<Socket> socket)
{
    // NS_LOG_UNCOND("QuicKohServer::HandleRead");


    Ptr<Packet> packet;
        Address from;


    while ((packet = socket->RecvFrom(from))) {
        // 연결 종료 (0 바이트)
        if (packet->GetSize() == 0) {
            NS_LOG_UNCOND("👋 Server: Client disconnected.");
            // 소켓 닫고 클라이언트 정보 제거
            socket->Close();
            break;
        }
        uint32_t receivedSize = packet->GetSize();
        packet->RemoveAllPacketTags ();
        packet->RemoveAllByteTags ();




            SeqTsHeader seqTs;
            uint32_t currentSequenceNumber = seqTs.GetSeq();

                std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
                          << InetSocketAddress::ConvertFrom(from).GetIpv4()
                          << " port: " << InetSocketAddress::ConvertFrom(from).GetPort()
                          << " Sequence Number: " << currentSequenceNumber
                          << " Uid: " << packet->GetUid() << " TXtime: " << seqTs.GetTs()
                          << " RXtime: " << Simulator::Now()
                          << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;



            m_received++;

            SendPacket(socket, from, std::string("good"));
            std::cout << "sent to client - socket ptr: " << socket << std::endl;
        }
    }


void QuicKohServer::SendPacket(Ptr<Socket> socket, Address from, const std::string &message)
    {
        Ptr<Packet> packet = Create<Packet>((uint8_t*) message.c_str(), message.length());
        socket->SendTo(packet,0,from);
    }

}
 // Namespace ns3
