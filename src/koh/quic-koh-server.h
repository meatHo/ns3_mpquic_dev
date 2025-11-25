//
// Created by loapp on 25. 8. 18.
//

#ifndef QUIC_KOH_SERVER_H
#define QUIC_KOH_SERVER_H

#endif
/*
 * Copyright (c) 2007,2008,2009 INRIA, UDCAST
 */

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/packet-loss-counter.h"
#include "ns3/ptr.h"
#include <ns3/traced-callback.h>
#include "kohTag.h"

namespace ns3
{

class QuicKohServer : public Application
{
  public:
    static TypeId GetTypeId();

    QuicKohServer();

    ~QuicKohServer() override {};

    /**
     * \brief Returns the number of lost packets
     * \return the number of lost packets
     */
    uint32_t GetLost(void) const;

    /**
     * \brief Returns the number of received packets
     * \return the number of received packets
     */
    uint64_t GetReceived(void) const;

    void SetPacketWindowSize(uint16_t size);

    uint16_t GetPacketWindowSize() const;

    uint32_t GetRecvCount(uint16_t ueId);
    double GetLatency(uint16_t ueId);
    void clearCount(uint16_t ueId);
    uint64_t GetTotalRecv();

  private:
    std::map<uint16_t, clientInfos> clients;
    void StartApplication() override;

    void StopApplication() override;

    void SendPacket(Ptr<Socket> socket,  Address from, const std::string& message);

    void HandleRead(Ptr<Socket> socket);
    void HandleAccept(Ptr<Socket> newSocket, const Address& from);

    void SendUeStats(uint16_t ueId);
    void Send();

    uint16_t m_port; //!< Port on which we listen for incoming packets.
    // uint8_t m_tos; //!< The packets Type of Service
    Ptr<Socket> m_socket;            //!< IPv4 Socket
    Ptr<Socket> m_socket6;           //!< IPv6 Socket
    uint64_t m_totalReceived;        //!< Number of received packets
    PacketLossCounter m_lossCounter; //!< Lost packet counter
    uint16_t m_nextClientId;
    uint64_t m_totalRx;
    Time m_txTs; //!< Time at which the last packet with header was received

    /// Callbacks for tracing the packet Rx events
    TracedCallback<Ptr<const Packet>> m_rxTrace;
    std::map<uint16_t, uint32_t> m_recvPerUe;
    std::map<uint16_t, double> m_latencySumPerUe;
    std::map<uint16_t, uint32_t> m_latencyCountPerUe;

    /// Callbacks for tracing the packet Rx events, includes source and destination addresses
    TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_rxTraceWithAddresses;
};
} // namespace ns3
