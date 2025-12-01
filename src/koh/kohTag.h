#ifndef KOH_TAG_H
#define KOH_TAG_H

#include "ns3/tag.h"
#include "ns3/header.h" // KohMetadata를 위해 추가
#include "ns3/nstime.h"
#include "string"
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

enum InterfaceType {
    UU = 0,
    SL = 1
};

enum frameType
{
    IDR,
    P,
    B
};


struct frameData
{
    uint32_t frameNum;
    frameType type;
    uint32_t size; // 바이트 단위 크기
};

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
    // 생성자: m_data 내부 변수들도 0 또는 기본값으로 초기화
    KohMetadata() : m_txTime(Seconds(0)) {
        m_data.frameNum = 0;
        m_data.type = P; // 기본값 설정 (필요시 IDR 등으로 변경)
        m_data.size = 0;
    }


    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::KohMetadata")
            .SetParent<Header>()
            .AddConstructor<KohMetadata>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override {
        return GetTypeId();
    }

    // --- 멤버 변수 Getter/Setter (구조체 m_data 접근) ---

    void SetTxTime(Time time) {
        m_txTime = time;
    }
    Time GetTxTime() const {
        return m_txTime;
    }

    // 구조체 통째로 설정
    void SetFrameData(frameData data) {
        m_data = data;
    }
    // 구조체 통째로 반환
    frameData GetFrameData() const {
        return m_data;
    }

    // 개별 필드 설정 편의 함수들
    void SetFrameNum(uint32_t num) {
        m_data.frameNum = num;
    }
    uint32_t GetFrameNum() const {
        return m_data.frameNum;
    }

    void SetFrameType(frameType type) {
        m_data.type = type;
    }

    frameType GetFrameType() const {
        return m_data.type;
    }

    std::string FrameTypeToString(frameType type) const {
        switch(type) {
        case IDR: return "IDR";
        case P:   return "P";
        case B:   return "B";
        default:  return "Unknown";
        }
    }

    std::string GetFrameTypeString() const {
        return FrameTypeToString(m_data.type);
    }

    void SetFrameSize(uint32_t size) {
        m_data.size = size;
    }
    uint32_t GetFrameSize() const {
        return m_data.size;
    }

    // --- Header 가상 함수 오버라이드 ---

    // 헤더의 직렬화된 크기 반환
    // Time(8) + FrameNum(4) + Type(4) + Size(4) = 20 bytes
    // *Type은 enum이지만 안전하게 uint32_t로 전송한다고 가정
    uint32_t GetSerializedSize() const override {
        return 8 + 4 + 4 + 4;
    }

    // 직렬화 (Serialize): 데이터를 버퍼에 씀
    void Serialize(Buffer::Iterator start) const override {
        start.WriteHtonU64(m_txTime.GetNanoSeconds());

        // m_data 구조체 내용 직렬화
        start.WriteHtonU32(m_data.frameNum);
        start.WriteHtonU32(static_cast<uint32_t>(m_data.type)); // Enum -> uint32 변환
        start.WriteHtonU32(m_data.size);
    }

    // 역직렬화 (Deserialize): 버퍼에서 데이터를 읽음
    uint32_t Deserialize(Buffer::Iterator start) override {
        m_txTime = NanoSeconds(start.ReadNtohU64());

        // m_data 구조체 내용 복원
        m_data.frameNum = start.ReadNtohU32();
        // uint32 -> Enum 강제 형변환
        m_data.type = static_cast<frameType>(start.ReadNtohU32());
        m_data.size = start.ReadNtohU32();

        return GetSerializedSize();
    }

    std::string toString() const {
        std::stringstream ss;
        ss << "TxTime=" << m_txTime
           << ", FrameNum=" << m_data.frameNum
           << ", Type=" << GetFrameTypeString()
           << ", Size=" << m_data.size;
        return ss.str();
    }

    void Print(std::ostream &os) const override {

    }

private:
    Time m_txTime;      // 전송 시간
    frameData m_data;   // 프레임 정보 구조체
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