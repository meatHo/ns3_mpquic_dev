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
#include "ns3/ipv4-address.h"
#include "ns3/internet-module.h"
#include "ns3/ptr.h"
#include "ns3/packet-loss-counter.h"
#include <ns3/traced-callback.h>

namespace ns3 {
    struct clientInfo {
        Address address;
        uint32_t lastSequenceNum;
        Time connectionTime;
        uint32_t RTT;
        float_t packetLossRate;
        uint64_t totalBytesReceived;
    };

    class QuicKohServer : public Application {
    public:
        static TypeId GetTypeId();

        QuicKohServer();

        ~QuicKohServer() override {
        };

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

    private:
        std::map<uint16_t, clientInfo> clients;

        void StartApplication() override;

        void StopApplication() override;

        void SendPacket(uint16_t clientId, std::string message);

        void HandleRead(Ptr<Socket> socket);

        uint16_t m_port; //!< Port on which we listen for incoming packets.
        // uint8_t m_tos; //!< The packets Type of Service
        Ptr<Socket> m_socket;            //!< IPv4 Socket
        Ptr<Socket> m_socket6; //!< IPv6 Socket
        uint64_t m_received; //!< Number of received packets
        PacketLossCounter m_lossCounter; //!< Lost packet counter
        uint16_t m_nextClientId;

        Time m_txTs; //!< Time at which the last packet with header was received

        /// Callbacks for tracing the packet Rx events
        TracedCallback<Ptr<const Packet> > m_rxTrace;

        /// Callbacks for tracing the packet Rx events, includes source and destination addresses
        TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses;
    };
} // namespace ns3
