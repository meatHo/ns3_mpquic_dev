#ifndef KOH_TAG_H
#define KOH_TAG_H

#include "ns3/tag.h"
#include "ns3/header.h" // KohMetadata를 위해 추가
#include "ns3/nstime.h"
#include "ns3/internet-module.h"

namespace ns3 {

// ... 기존 KStats, clientInfos 구조체 ...

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

    std::vector<uint8_t> buffer;
    uint32_t expectedBytes = 0;

    bool isReassembly = false;
    Time txTime;
    uint32_t packtNum;
};

/*
 * =================================================================================
 *                                KohMetadata Header
 * =================================================================================
 * 이 헤더는 클라이언트와 서버 간에 전송 시간을 포함한 메타데이터를 전달하기 위해 사용됩니다.
 * ns3::Tag와 달리 ns3::Header를 상속받아 실제 패킷 데이터에 포함되어 네트워크로 전송됩니다.
 */
class KohMetadata : public Header {
public:
    KohMetadata() : m_txTime(Seconds(0)), m_packetSize(0), m_packetNum(0) {}

    // TypeId 관리를 위한 정적 메소드
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::KohMetadata")
            .SetParent<Header>()
            .AddConstructor<KohMetadata>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override {
        return GetTypeId();
    }

    // --- 멤버 변수 Getter/Setter ---
    void SetTxTime(Time time) {
        m_txTime = time;
    }

    Time GetTxTime() const {
        return m_txTime;
    }

    void SetPacketSize(uint32_t size) {
        m_packetSize = size;
    }

    uint32_t GetPacketSize() const {
        return m_packetSize;
    }

    void SetPacketNum(uint32_t num) {
        m_packetNum = num;
    }

    uint32_t GetPacketNum() const {
        return m_packetNum;
    }

    // --- Header 가상 함수 오버라이드 ---
    // 헤더의 직렬화된 크기를 반환합니다. (시간 8바이트 + 크기 4바이트)
    uint32_t GetSerializedSize() const override {
        return 16;
    }

    // 멤버 변수들을 버퍼에 직렬화(쓰기)합니다. (패킷 전송 시 호출됨)
    void Serialize(Buffer::Iterator start) const override {
        start.WriteHtonU64(m_txTime.GetNanoSeconds());
        start.WriteHtonU32(m_packetSize);
        start.WriteHtonU32(m_packetNum);
    }

    // 버퍼로부터 데이터를 읽어 멤버 변수들을 역직렬화(복원)합니다. (패킷 수신 시 호출됨)
    uint32_t Deserialize(Buffer::Iterator start) override {
        m_txTime = NanoSeconds(start.ReadNtohU64());
        m_packetSize = start.ReadNtohU32();
        m_packetNum = start.ReadNtohU32();
        return GetSerializedSize(); // 읽은 바이트 수를 반환
    }

    // 디버깅을 위한 출력 함수
    void Print(std::ostream &os) const override {
        os << "TxTime=" << m_txTime << ", PacketSize=" << m_packetSize;
    }

private:
    Time m_txTime;        // 전송 시간
    uint32_t m_packetSize;  // 전체 패킷 크기
    uint32_t m_packetNum;  // 전체 패킷순서
};


// ... 기존 KohTag 클래스 ...

class KohTag : public Tag {
public:
    KohTag() : ueId(0), txTime(Seconds(0)), packetId(0) {}
    KohTag(uint16_t i) : ueId(i), txTime(Seconds(0)), packetId(0) {}

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
        i.WriteU32(packetId);
    }

    void Deserialize(TagBuffer i) override {
        ueId = i.ReadU16();
        txTime = Seconds(i.ReadDouble());
        packetId = i.ReadU32();
    }

    uint32_t GetSerializedSize() const override { return 10 + 4; }

    void Print(std::ostream &os) const override {
        os << "UE=" << ueId << " Tx=" << txTime.GetSeconds() << " PacketID=" << packetId;
    }

    // 멤버 순서 중요!
    uint16_t ueId;
    Time txTime;
    uint32_t packetId;
};

} // namespace ns3

#endif // KOH_TAG_H