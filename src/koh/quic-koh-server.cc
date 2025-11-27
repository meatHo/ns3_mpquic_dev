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
    Ptr<Packet> packet;
    Address from;
    Address localAddress;

    while ((packet = socket->RecvFrom(from)))
    {
        if (packet->GetSize() == 0)
        {
            continue;
        }

        // 1. 클라이언트 식별 및 상태 정보 생성/가져오기
        auto it = std::find_if(clients.begin(), clients.end(),
                               [&](const std::pair<uint16_t, clientInfos>& clientPair) {
                                   return clientPair.second.address == from;
                               });
        
        uint16_t clientId;
        if (it == clients.end())
        {
            NS_LOG_UNCOND("New client detected: " << InetSocketAddress::ConvertFrom(from).GetIpv4());
            clientId = m_nextClientId++;
            clients[clientId] = {}; // clientInfos 구조체를 0으로 초기화
            clients[clientId].address = from;
            clients[clientId].socket = socket;
        }
        else
        {
            clientId = it->first;
        }
        clientInfos& client = clients[clientId];

        // 2. 수신된 데이터를 클라이언트의 내부 버퍼에 추가
        uint8_t* bufferData = new uint8_t[packet->GetSize()];
        packet->CopyData(bufferData, packet->GetSize());
        client.buffer.insert(client.buffer.end(), bufferData, bufferData + packet->GetSize());
        delete[] bufferData;
        
        socket->GetSockName(localAddress);
        m_rxTraceWithAddresses(packet, from, localAddress);

        // 3. 버퍼에 완전한 메시지가 있는지 확인하고 처리하는 루프
        while (true)
        {
            // === 상태: 새로운 메시지 시작을 기다리는 중 ===
            if (client.expectedBytes == 0)
            {
                // A. Uu 메시지인지 확인 (KohMetadata 헤더 확인)
                // KohMetadata 크기(16바이트)보다 버퍼가 크거나 같아야 확인 가능
                if (client.buffer.size() >= 16)
                {
                    // 버퍼의 앞부분으로 임시 패킷을 만들어 헤더를 확인(Peek)
                    Ptr<Packet> tempPacket = Create<Packet>(client.buffer.data(), 16);
                    KohMetadata metadata;
                    if (tempPacket->PeekHeader(metadata))
                    {
                        // Uu 패킷 스트림 시작 감지
                        NS_LOG_INFO("Detected Uu packet stream start. Total size: " << metadata.GetPacketSize());
                        client.isReassembly = true; // Use user's field name
                        client.expectedBytes = metadata.GetPacketSize(); // 전체 패킷 크기 저장
                        client.txTime = metadata.GetTxTime(); // Use user's field name
                        client.packtNum = metadata.GetPacketNum(); // Use user's field name
                    }
                }

                // 새로운 메시지 시작을 감지하지 못했다면, 데이터가 더 필요하므로 루프 종료
                if (client.expectedBytes == 0)
                {
                    break;
                }
            }

            // === 상태: 특정 크기의 메시지를 재조립하는 중 ===
            if (client.expectedBytes > 0)
            {
                uint32_t requiredBytesInBuffer;
                if (client.isReassembly) // If it's a Uu reassembly
                {
                    // Uu의 경우, expectedBytes가 전체 크기(헤더 포함)이므로 버퍼에 그만큼 있어야 함
                    requiredBytesInBuffer = client.expectedBytes;
                }
                
                // C. 버퍼에 충분한 데이터가 있는지 확인
                if (client.buffer.size() >= requiredBytesInBuffer)
                {
                    // 완전한 메시지 하나를 처리할 데이터가 모임
                    Ptr<Packet> completePacket;

                    if (client.isReassembly) // If it's a Uu reassembly
                    {
                        // Uu 패킷 재조립 완료 및 처리
                        NS_LOG_UNCOND("Uu packet reassembly complete. Total size: " << requiredBytesInBuffer<<" Packet num: "<<client.packtNum);
                        
                        // 버퍼에서 전체 패킷(헤더+몸통) 생성
                        completePacket = Create<Packet>(client.buffer.data(), requiredBytesInBuffer);
                        
                        // 헤더를 제거하면서 정보 다시 확인
                        KohMetadata metadata;
                        completePacket->RemoveHeader(metadata); // This also verifies it's there.

                        Time delay = Simulator::Now() - metadata.GetTxTime();
                        NS_LOG_UNCOND("Processed Uu packet. Seq: " << metadata.GetPacketNum()
                                      << ", Latency: " << delay.GetMilliSeconds() << "ms");
                        
                        m_totalRx += requiredBytesInBuffer;
                        m_totalReceived++;

                        // Remove the full packet from client buffer
                        client.buffer.erase(client.buffer.begin(), client.buffer.begin() + requiredBytesInBuffer);
                    }


                    // 상태 초기화하여 다음 메시지를 기다리도록 설정
                    client.expectedBytes = 0;
                    client.isReassembly = false; // Reset the reassembly flag
                    
                    // 버퍼에 다른 메시지가 더 있을 수 있으므로 루프 계속
                    continue;
                }
                else
                {
                    // 아직 데이터가 부족함. 루프를 중단하고 다음 데이터 수신을 기다림
                    break;
                }
            } 
        } // end while(true)
    } // end while(RecvFrom)
}


// void
// QuicKohServer::HandleRead(Ptr<Socket> socket)
// {
//     NS_LOG_UNCOND("QuicKohServer::HandleRead");
//     Ptr<Packet> packet;
//     Address from;
//     Address localAddress;
//
//     while ((packet = socket->RecvFrom(from)))
//     {
//         bool clientFound = false;
//         uint16_t clientId;
//         for (const auto& pair : clients)
//         {
//             if (pair.second.address == from)
//             {
//                 clientFound = true;
//                 clientId = pair.first;
//                 break;
//             }
//         }
//
//         InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
//
//         if (!clientFound)
//         {
//             NS_LOG_UNCOND("new client detected");
//             uint16_t newId = m_nextClientId++;
//             clientInfos newClient;
//             newClient.address = from;
//             newClient.lastSequenceNum = 0;
//             newClient.connectionTime = Simulator::Now();
//             newClient.packetLossRate = 0;
//             newClient.RTT = 0;
//             newClient.socket = socket;
//             newClient.totalBytesReceived = 0;
//
//             // [추가] 버퍼링을 위한 초기화 (구조체에 이 멤버들이 있어야 함)
//             newClient.expectedBytes = 0;
//             newClient.buffer.clear();
//
//             clients[newId] = newClient;
//             clientId = newId;
//         }
//
//         // 편의를 위해 클라이언트 참조
//         clientInfos& client = clients[clientId];
//
//
//         // Check for KohMetadata first
//         KohMetadata metadata;
//         if (packet->GetSize()>metadata.GetSerializedSize())
//         {
//             if (packet->PeekHeader(metadata))
//             {
//                 packet->RemoveHeader(metadata);
//                 socket->GetSockName(localAddress);
//                 m_rxTrace(packet);
//                 m_rxTraceWithAddresses(packet, from, localAddress);
//
//                 Time txTime = metadata.GetTxTime();
//                 Time now = Simulator::Now();
//                 Time delay = now - txTime;
//
//                 NS_LOG_UNCOND("Received Uu packet with KohMetadata. "
//                               << "From: " << InetSocketAddress::ConvertFrom(from).GetIpv4()
//                               << ", Size: " << metadata.GetPacketSize()
//                               << ", Seq: " << metadata.GetPacketNum()
//                               << ", Latency: " << delay.GetMilliSeconds() << "ms");
//
//                 m_totalRx += metadata.GetPacketSize();
//                 m_totalReceived++;
//                 // Bypassing the old logic for these packets.
//                 continue; // Continue to the next packet
//             }
//         }
//
//
//
//         // ---------------------------------------------------------
//         // [B] 물리/MAC 계층 통계 처리 (Tag 처리)
//         // 주의: Tag는 버퍼로 변환 시 사라지므로, 원본 packet에서 처리해야 함
//         // ---------------------------------------------------------
//         socket->GetSockName(localAddress);
//         m_rxTrace(packet);
//         m_rxTraceWithAddresses(packet, from, localAddress);
//
//         // ---------------------------------------------------------
//         // [C] 데이터 버퍼링 (수신 패킷을 클라이언트 버퍼에 추가)
//         // ---------------------------------------------------------
//         uint8_t* bufferData = new uint8_t[packet->GetSize()];
//         packet->CopyData(bufferData, packet->GetSize());
//         client.buffer.insert(client.buffer.end(), bufferData, bufferData + packet->GetSize());
//         delete[] bufferData;
//
//         // ---------------------------------------------------------
//         // [D] 패킷 조립 및 애플리케이션 로직 처리 루프
//         // ---------------------------------------------------------
//         while (true)
//         {
//             // 1. 메시지 크기(Header) 확인 단계
//             if (client.expectedBytes == 0)
//             {
//                 // 크기 정보(4바이트)가 아직 덜 왔으면 대기
//                 if (client.buffer.size() < sizeof(uint32_t))
//                 {
//                     break;
//                 }
//
//                 // 4바이트를 읽어 전체 메시지 크기 설정
//                 memcpy(&client.expectedBytes, client.buffer.data(), sizeof(uint32_t));
//
//                 // 버퍼에서 크기 정보(헤더) 삭제
//                 client.buffer.erase(client.buffer.begin(), client.buffer.begin() + sizeof(uint32_t));
//             }
//
//             // 2. 메시지 본문(Body) 확인 단계
//             if (client.expectedBytes > 0 && client.buffer.size() >= client.expectedBytes)
//             {
//                 // 완전한 패킷 생성 (ns-3 Packet으로 재조립)
//                 Ptr<Packet> completePacket = Create<Packet>(client.buffer.data(), client.expectedBytes);
//                 NS_LOG_UNCOND(">>> [Reassembly Complete] Full Packet Received! Size: " << client.expectedBytes << " bytes");
//                 // -----------------------------------------------------
//                 // [E] 기존 애플리케이션 로직 (Header 파싱 및 로그)
//                 // -----------------------------------------------------
//                 if (completePacket->GetSize() > 12) // SeqTsHeader 크기 체크 등
//                 {
//                     uint32_t receivedSize = completePacket->GetSize();
//                     m_totalRx += receivedSize; // 애플리케이션 레벨 수신량
//                     NS_LOG_UNCOND("total rx : " << m_totalRx);
//
//                     SeqTsHeader seqTs;
//                     // 조립된 패킷에서 헤더 제거 및 정보 읽기
//                     completePacket->RemoveHeader(seqTs);
//
//                     NS_LOG_UNCOND("ip :" << addr.GetIpv4() << "  size :" << completePacket->GetSize() << "  seq :" << seqTs.GetSeq());
//
//                     uint32_t currentSequenceNumber = seqTs.GetSeq();
//
//                     if (InetSocketAddress::IsMatchingType(from))
//                     {
//                         // IPv4 로그 (필요시 주석 해제)
//                     }
//                     else if (Inet6SocketAddress::IsMatchingType(from))
//                     {
//                         std::cout << "TraceDelay: RX " << receivedSize << " bytes from "
//                                   << Inet6SocketAddress::ConvertFrom(from).GetIpv6()
//                                   << " port: " << Inet6SocketAddress::ConvertFrom(from).GetPort()
//                                   << " Sequence Number: " << currentSequenceNumber
//                                   << " Uid: " << completePacket->GetUid() // 주의: 재조립된 패킷이라 UID는 새로 발급됨
//                                   << " TXtime: " << seqTs.GetTs()
//                                   << " RXtime: " << Simulator::Now()
//                                   << " Delay: " << Simulator::Now() - seqTs.GetTs() << std::endl;
//                     }
//
//                     m_lossCounter.NotifyReceived(currentSequenceNumber);
//                     m_totalReceived++;
//                 }
//
//                 // 처리 완료된 데이터를 버퍼에서 제거
//                 client.buffer.erase(client.buffer.begin(), client.buffer.begin() + client.expectedBytes);
//
//                 // 다음 메시지 처리를 위해 초기화
//                 client.expectedBytes = 0;
//
//                 // 버퍼에 데이터가 더 남아있을 수 있으므로 continue (while문 다시 실행)
//                 continue;
//             }
//             else
//             {
//                 // 데이터가 아직 부족하면 루프 탈출 후 다음 소켓 수신 대기
//                 break;
//             }
//         } // end of while(true)
//     } // end of while(RecvFrom)
// }

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