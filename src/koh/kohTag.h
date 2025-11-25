#ifndef KOH_TAG_H
#define KOH_TAG_H

#include "ns3/tag.h"
#include "ns3/nstime.h"
#include "ns3/internet-module.h"

namespace ns3 {

struct KStats
{
    uint16_t ueId;
    uint32_t recvCount;
    uint32_t sentCount;
    double avgLatency;
} __attribute__((packed));

struct clientInfos
{
    Address address;
    uint32_t lastSequenceNum;
    Time connectionTime;
    uint32_t RTT;
    float_t packetLossRate;
    uint64_t totalBytesReceived;
    Ptr<Socket> socket;
};

class KohTag : public Tag {
public:
    KohTag() : ueId(0), txTime(Seconds(0)) {}
    KohTag(uint16_t i) : ueId(i), txTime(Seconds(0)) {}

    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::KohTag")
            .SetParent<Tag>()
            .AddConstructor<KohTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }

    void Serialize(TagBuffer i) const override {
        i.WriteU16(ueId);
        i.WriteDouble(txTime.GetSeconds());
    }

    void Deserialize(TagBuffer i) override {
        ueId = i.ReadU16();
        txTime = Seconds(i.ReadDouble());
    }

    uint32_t GetSerializedSize() const override { return 10; }

    void Print(std::ostream &os) const override {
        os << "UE=" << ueId << " Tx=" << txTime.GetSeconds();
    }

    // 멤버 순서 중요!
    uint16_t ueId;
    Time txTime;
};

} // namespace ns3

#endif // KOH_TAG_H
