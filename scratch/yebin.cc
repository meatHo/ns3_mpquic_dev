// yebin code
#include "/home/kiho/ns-3-quic/contrib/nr/examples/nr-v2x-examples/v2x-kpi.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/eps-bearer.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/lte-sl-tft.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-phy-rx-trace.h" // PhyRsrpSinr 구조체 정의
#include "ns3/nr-sl-helper.h"
#include "ns3/opengym-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"

#include <cmath>
#include <iomanip>

using namespace ns3;

// Sl 카운터 (추가!)
uint64_t g_totalRxBitsSl = 0;
uint32_t g_rxPacketsSl = 0;

// === 전역 변수 ===
ApplicationContainer g_uuClientApps;
Ptr<Socket> g_slSendSocket;
InetSocketAddress g_remote = InetSocketAddress(Ipv4Address::GetAny(), 0);

static std::set<uint32_t> activatedUes;
// === Global DB & Stats ===
static std::unique_ptr<SQLiteOutput> g_db; // 포인터로 선언
static UeRlcRxOutputStats g_rlcRxStats;

// === Global vars for PHY measurements ===
static double g_lastRsrp = -120.0; // 초기값 dBm
static double g_lastSinr = 0.0;    // 초기값 dB

// === Global vars for 모드 전환 횟수 ===
static uint32_t g_switchCount = 0;

// === Global vars for Throughput (그대로 유지) ===
static double g_totalRxBitsUu = 0.0;
static uint32_t g_rxPacketsUu = 0;

void NotifySlRx(Ptr<Socket> socket);
void RxTraceCallback(ns3::Ptr<const ns3::Packet> packet);

void TxOnlyCallback(Ptr<const Packet> packet);
void TxWithAddrCallback(Ptr<const Packet> packet, const Address& from, const Address& to);
void TxWithSeqCallback(Ptr<const Packet> packet,
                       const Address& from,
                       const Address& to,
                       const SeqTsSizeHeader& header);

Ptr<NrUeNetDevice> GetUeNetDevice(Ptr<Node> node);

void
Ipv4PacketTraceAtRsu(Ptr<const Packet> packet, Ptr<Ipv4> Ipv4, uint32_t interfaceIndex)
{
    Ipv4Header Ipv4Header;
    packet->PeekHeader(Ipv4Header);

    std::cout << "[RSU Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

void
SendUdpPacket(Ptr<Socket> socket, uint32_t size)
{
    Ptr<Packet> packet = Create<Packet>(size);
    socket->Send(packet);
    std::cout << ">>> SL Client sent " << size << "B @ " << Simulator::Now().GetSeconds() << "s"
              << std::endl;
}

// --- 수신 Trace 콜백 ---
void
RxTraceCallback(Ptr<const Packet> packet)
{
    std::cout << ">>> Trace callback fired, packet size=" << packet->GetSize() << " @ "
              << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_totalRxBitsSl += packet->GetSize() * 8;
    g_rxPacketsSl++;
}

Ptr<NrUeNetDevice>
GetUeNetDevice(Ptr<Node> node)
{
    for (uint32_t i = 0; i < node->GetNDevices(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(node->GetDevice(i));
        if (ueDev)
        {
            return ueDev; // 찾으면 반환
        }
    }
    return nullptr; // 없으면 nullptr
}

// --- 수신 콜백 ---
void
NotifySlRx(Ptr<Socket> socket)
{
    std::cout << ">>> NotifySlRx TRIGGERED @ " << Simulator::Now().GetSeconds() << "s" << std::endl;

    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom(from)))
    {
        std::cout << ">>> RSU received " << packet->GetSize() << "B from "
                  << InetSocketAddress::ConvertFrom(from).GetIpv4() << " @ "
                  << Simulator::Now().GetSeconds() << "s" << std::endl;

        g_totalRxBitsSl += packet->GetSize() * 8;
        g_rxPacketsSl++;
    }
}

// === 패킷 송신 함수 ===
void
SendSlPacket(Ptr<Socket> socket, Ipv4Address dstAddr, uint16_t dstPort)
{
    Ptr<Packet> pkt = Create<Packet>(200);
    socket->SendTo(pkt, 0, InetSocketAddress(dstAddr, dstPort)); // SendTo 사용

    std::cout << ">>> SL Client sent 200B to " << dstAddr << ":" << dstPort << " @ "
              << Simulator::Now().GetSeconds() << "s" << std::endl;
}

// --- Throughput 출력 (1초마다) ---
void
PrintThroughput()
{
    double time = Simulator::Now().GetSeconds();
    double thrUu = (time > 0) ? g_totalRxBitsUu / time / 1e6 : 0;
    double thrSl = (time > 0) ? g_totalRxBitsSl / time / 1e6 : 0;

    std::cout << "[Throughput " << time << "s] "
              << "Uu=" << thrUu << " Mbps, "
              << "SL=" << thrSl << " Mbps" << std::endl;

    Simulator::Schedule(Seconds(1.0), &PrintThroughput);
}

// === Summary 출력 함수 ===
void
PrintSummary(Ptr<PacketSink> sink, Time simTime)
{
    std::cout << ">>> [SUMMARY] Total received by PacketSink = " << sink->GetTotalRx() << " bytes"
              << std::endl;

    double avgThroughput = sink->GetTotalRx() * 8.0 / simTime.GetSeconds() / 1e6;
    std::cout << ">>> [SUMMARY] Average throughput = " << avgThroughput << " Mbps" << std::endl;
}

void
NotifyUuRx(Ptr<const Packet> packet, const Address& from)
{
    if (!packet)
        return;

    uint32_t pktSize = packet->GetSize();
    g_totalRxBitsUu += pktSize * 8;
    g_rxPacketsUu++;

    if (InetSocketAddress::IsMatchingType(from))
    {
        Ipv4Address srcAddr = InetSocketAddress::ConvertFrom(from).GetIpv4();
        std::cout << "[Uu-Rx] size=" << pktSize << "B from=" << srcAddr << " @ "
                  << Simulator::Now().GetSeconds() << "s" << std::endl;
    }
    else
    {
        std::cout << "[Uu-Rx] size=" << pktSize << "B from=UNKNOWN" << std::endl;
    }
}

// === Uu RLC 수신 콜백 ===
void
NotifyUuRlcPduRx(uint64_t /*imsi*/,
                 uint16_t /*rnti*/,
                 uint16_t /*txRnti*/,
                 uint8_t /*lcid*/,
                 uint32_t size,
                 double /*delay*/)
{
    g_rxPacketsUu++;
    g_totalRxBitsUu += size * 8;
}

NS_LOG_COMPONENT_DEFINE("NrV2xWestToEastHighway");

/*
 각 계층(APP, MAC, PHY 등)에서 발생하는 통신 기록을 받아서 DB에 저장하는 함수를 모아둔 전역 함수들
 UE(MAC 계층)가 sidelink 제어 메시지(SCI 1-A) 보낼 때 기록하는 함수 → "차량이 제어 메시지 보냈다"
 저장
 */
void
NotifySlPscchScheduling(UeMacPscchTxOutputStats* pscchStats,
                        const SlPscchUeMacStatParameters pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/* UE(MAC 계층)가 데이터(PSSCH, SCI 2-A)를 보낼 때 기록하는 함수 → "차량이 데이터를 보냈다" 저장 */
void
NotifySlPsschScheduling(UeMacPsschTxOutputStats* psschStats,
                        const SlPsschUeMacStatParameters psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

/**
PHY 계층에서 UE가 제어 메시지(SCI 1-A)를 받았을 때 기록하는 함수 → "차량이 제어 메시지를 받았다"
저장
 */
void
NotifySlPscchRx(UePhyPscchRxOutputStats* pscchStats,
                const SlRxCtrlPacketTraceParams pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/* PHY 계층에서 UE가 데이터 (PSSCH, SCI 2-A)를 받았을 때 기록하는 함수 → "차량이 데이터를 받았다"
 * 저장 */
void
NotifySlPsschRx(UePhyPsschRxOutputStats* psschStats,
                const SlRxDataPacketTraceParams psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

void
RsrpCallback(std::string context,
             uint16_t cellId,
             uint16_t rnti,
             uint16_t bwpId,
             double rsrp,
             uint8_t layer)
{
    // RSRP: W → mW → dBm
    g_lastRsrp = 10 * std::log10(rsrp * 1000.0);
    std::cout << ">>> UE Trace: raw RSRP=" << rsrp << " W, converted=" << g_lastRsrp << " dBm"
              << std::endl;
}

void
SinrCallback(std::string context, uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    // SINR 변환 (리니어 값일 경우)
    g_lastSinr = 10 * std::log10(sinr);
    std::cout << ">>> UE Trace: raw SINR=" << sinr << ", converted=" << g_lastSinr << " dB"
              << std::endl;
}

/* 애플리케이션 계층에서 차량 간 주고받은 패킷을 기록하는 함수 → "누가 누구에게 몇 번 패킷을 보냈다"
 * 저장 */
void
UePacketTraceDb(UeToUePktTxRxOutputStats* stats,
                Ptr<Node> node,
                const Address& localAddrs,
                std::string txRx,
                Ptr<const Packet> p,
                const Address& srcAddrs,
                const Address& dstAddrs,
                const SeqTsSizeHeader& seqTsSizeHeader)
{
    uint32_t nodeId = node->GetId();
    uint64_t imsi = node->GetDevice(0)->GetObject<NrUeNetDevice>()->GetImsi();
    uint32_t seq = seqTsSizeHeader.GetSeq();
    uint32_t pktSize = p->GetSize() + seqTsSizeHeader.GetSerializedSize();

    stats->Save(txRx, localAddrs, nodeId, imsi, pktSize, srcAddrs, dstAddrs, seq);
}

/* RLC 계층에서 받은 패킷 크기와 지연시간을 기록하는 함수 → "데이터가 얼마나 늦게 도착했는지" 저장
 */

void
NotifySlRlcPduRx(UeRlcRxOutputStats* stats,
                 uint64_t imsi,
                 uint16_t rnti,
                 uint16_t txRnti,
                 uint8_t lcid,
                 uint32_t rxPduSize,
                 double delay)
{
    if (!stats)
        return;
    stats->Save(imsi, rnti, txRnti, lcid, rxPduSize, delay);

    g_rxPacketsSl++;
    g_totalRxBitsSl += (rxPduSize * 8);

    std::cout << "[SL-RLC] IMSI=" << imsi << " size=" << rxPduSize << " bytes"
              << " @ " << Simulator::Now().GetSeconds() << "s" << std::endl;
}

void
PrintUeInfo(NodeContainer ueNodes)
{
    uint8_t i = 0;
    NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
    for (auto it = ueNodes.Begin(); it != ueNodes.End(); ++it)
    {
        Ptr<Node> ueNode = *it;
        Ptr<MobilityModel> mob = ueNode->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector vel = mob->GetVelocity();
        NS_LOG_UNCOND("Vehicle " << std::to_string(++i));
        NS_LOG_UNCOND("UE Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z);
        NS_LOG_UNCOND("UE Velocity: x=" << vel.x << ", y=" << vel.y << ", z=" << vel.z << " (m/s)");
    }
    Simulator::Schedule(Seconds(1.0), &PrintUeInfo, ueNodes);
}

/* 고속도로 환경에서 차선/차량 배치와 속도를 설정하는 함수 → "도로 위에 차들" */
NodeContainer
InstallHighwayMobility(uint16_t totalLanes,
                       uint16_t numVehiclesPerLane,
                       uint16_t interVehicleDist,
                       uint16_t interLaneDist,
                       double speed)
{
    NS_ABORT_MSG_IF((numVehiclesPerLane * totalLanes) % totalLanes != 0,
                    "All the lanes must have equal number of UEs");

    NodeContainer ueNodes;

    ueNodes.Create(numVehiclesPerLane * totalLanes);

    std::cout << "Total UEs = " << ueNodes.GetN() << std::endl;

    Ptr<GridPositionAllocator> gridPositionAllocator;
    gridPositionAllocator = CreateObject<GridPositionAllocator>();
    gridPositionAllocator->SetAttribute("GridWidth", UintegerValue(numVehiclesPerLane));
    gridPositionAllocator->SetAttribute("MinX", DoubleValue(0.0));
    gridPositionAllocator->SetAttribute("MinY", DoubleValue(0.0));
    gridPositionAllocator->SetAttribute("Z", DoubleValue(1.6));
    gridPositionAllocator->SetAttribute("DeltaX", DoubleValue(interVehicleDist));
    gridPositionAllocator->SetAttribute("DeltaY", DoubleValue(interLaneDist));
    gridPositionAllocator->SetAttribute("LayoutType", EnumValue(GridPositionAllocator::ROW_FIRST));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.SetPositionAllocator(gridPositionAllocator);

    mobility.Install(ueNodes);

    for (int i = 0; i < totalLanes * numVehiclesPerLane; i++)
    {
        if (i == 0)
        {
            // Node 0 → RSU 역할 (속도 0으로 고정)
            ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
                Vector(0.0, 0.0, 0.0));
            std::cout << "Node " << i << " set as RSU (fixed position)" << std::endl;
        }
        else
        {
            // 나머지는 일반 차량
            ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
                Vector(speed, 0.0, 0.0));
        }
    }
    return ueNodes;
}

/* 차량 초기 위치를 gnuplot으로 그릴 수 있는 스크립트 작성 → 차 출발 위치 지도 만들기 */
void
WriteInitPosGnuScript(std::string posFilename)
{
    std::ofstream outFile;
    std::string filename = "gnu-script-" + posFilename;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }

    std::string pngFileName;
    pngFileName = posFilename.substr(0, posFilename.rfind('.'));
    outFile << "set terminal png" << std::endl;
    outFile << "set output \"" << pngFileName << ".png\"" << std::endl;
    outFile << "set style line 1 lc rgb 'black' ps 2 pt 7" << std::endl;
    outFile << "unset key" << std::endl;
    outFile << "set grid" << std::endl;
    outFile << "plot \"" << posFilename << "\" using 3:4 with points ls 1";
    outFile.close();
}

/* 차량 초기 위치를 파일로 저장 → 출발선 차량 좌표 저장 */
void
PrintUeInitPosToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << node->GetId() << " " << uedev->GetImsi() << " " << pos.x << " " << pos.y
                        << std::endl;
            }
        }
    }

    WriteInitPosGnuScript(filename);
}

/* 차량 이동 위치를 매초 파일에 기록 → 차가 달리는 궤적 기록 */
void
RecordMobility(bool FirstWrite, std::string fileName)
{
    std::ofstream outFile;
    if (FirstWrite)
    {
        outFile.open(fileName.c_str(), std::ios_base::out);
        FirstWrite = false;
    }
    else
    {
        outFile.open(fileName.c_str(), std::ios_base::app);
        outFile << std::endl;
        outFile << std::endl;
    }

    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << Simulator::Now().GetSeconds() << " " << node->GetId() << " "
                        << uedev->GetImsi() << " " << pos.x << " " << pos.y << std::endl;
            }
        }
    }

    Simulator::Schedule(Seconds(1), &RecordMobility, FirstWrite, fileName);
}

/* 차량 이동 궤적을 GIF로 볼 수 있는 gnuplot 스크립트 작성 → 차 달리는 모습 영상 만들기 */
void
WriteGifGnuScript(std::string MobilityFileName,
                  Time simTime,
                  double speed,
                  Ptr<Node> firstUeNode,
                  Ptr<Node> lastUeNode)
{
    std::ofstream outFile;
    std::string fileName = "gif-script-" + MobilityFileName;
    outFile.open(fileName.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << fileName);
        return;
    }
    outFile << "set term gif animate delay 100" << std::endl;
    std::string gifFileName;
    gifFileName = MobilityFileName.substr(0, MobilityFileName.rfind('.'));
    outFile << "set output \"" << gifFileName << ".gif"
            << "\"" << std::endl;
    outFile << "unset key" << std::endl;
    outFile << "set grid" << std::endl;

    Vector firstNodePos = firstUeNode->GetObject<MobilityModel>()->GetPosition();
    Vector LastNodePos = lastUeNode->GetObject<MobilityModel>()->GetPosition();
    double xRangeLower = firstNodePos.x - 10.0;
    double xRangeUpper = simTime.GetSeconds() * speed + LastNodePos.x;
    double yRangeLower = firstNodePos.y - 10.0;
    double yRangeUpper = LastNodePos.y + 10.0;
    outFile << "set xrange [" << xRangeLower << ":" << xRangeUpper << "]" << std::endl;
    outFile << "set yrange [" << yRangeLower << ":" << yRangeUpper << "]" << std::endl;
    outFile << "do for [i=0:" << simTime.GetSeconds() - 1 << "] {plot \"" << MobilityFileName
            << "\" index i using 4:5}" << std::endl;
}

/* 문자열로 주어진 0/1 코드(비트맵)를 실제 통신 시간표 형식으로 변환 → 문자열을 통신 시간표로 바꾸기
 */
void
GetSlBitmapFromString(std::string slBitMapString, std::vector<std::bitset<1>>& slBitMapVector)
{
    static std::unordered_map<std::string, uint8_t> lookupTable = {
        {"0", 0},
        {"1", 1},
    };

    std::stringstream ss(slBitMapString);
    std::string token;
    std::vector<std::string> extracted;

    while (std::getline(ss, token, '|'))
    {
        extracted.push_back(token);
    }

    for (const auto& v : extracted)
    {
        if (lookupTable.find(v) == lookupTable.end())
        {
            NS_FATAL_ERROR("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
        slBitMapVector.emplace_back(lookupTable[v] & 0x01);
    }
}

/* 차량의 IP 주소와 위치를 매칭해서 저장 → "어떤 IP가 어디에 있는지 기록" */
void
SavePositionPerIP(V2xKpi* v2xKpi)
{
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Ptr<Ipv4L3Protocol> ipv4Protocol = node->GetObject<Ipv4L3Protocol>();
                Ipv4InterfaceAddress addresses = ipv4Protocol->GetAddress(1, 0);
                std::ostringstream ueIpv4Addr;
                ueIpv4Addr.str("");
                ueIpv4Addr << addresses.GetLocal();
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                v2xKpi->FillPosPerIpMap(ueIpv4Addr.str(), pos);
            }
        }
    }
}

// SL 전용: UE1(Car) ↔ UE0(RSU) 연결
// === SL Attach (UE1 ↔ UE0만) ===

void
ScheduleNextStateRead(Time stepTime, Ptr<OpenGymInterface> openGymInterface)
{
    if (Simulator::Now().GetSeconds() >= Simulator::GetMaximumSimulationTime().GetSeconds())
        return; // 더 이상 step 예약하지 않음

    openGymInterface->NotifyCurrentState();
    Simulator::Schedule(stepTime, &ScheduleNextStateRead, stepTime, openGymInterface);
}

// observation space 콜백
Ptr<OpenGymSpace>
MyGetObservationSpace()
{
    std::vector<uint32_t> shape = {6}; // 상태 차원 = 5
    float low = -1000.0;               // 더 작은 값
    float high = 200.0;                // 더 큰 값
    std::string dtype = "float32";

    Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    return space;
}

// 전역 변수로 현재 모드 관리
static uint32_t g_currentMode = 0; // 0=KEEP, 1=gNB, 2=RSU

Ptr<OpenGymDataContainer>
MyGetObservation()
{
    Ptr<Node> ueNode = NodeList::GetNode(0);
    Vector pos = ueNode->GetObject<MobilityModel>()->GetPosition();
    float position = pos.x;

    float rsrp = g_lastRsrp;
    float sinr = g_lastSinr;
    uint32_t mode = 0;
    uint32_t density = NodeList::GetNNodes();

    std::vector<uint32_t> shape = {6}; // 차원 하나 늘림
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);

    box->AddValue((float)position);
    box->AddValue((float)rsrp);
    box->AddValue((float)sinr);
    box->AddValue((float)mode);
    box->AddValue((float)density);

    // === 추가된 부분 ===
    box->AddValue((float)g_rxPacketsSl);

    return box;
}

Ptr<OpenGymSpace>
MyGetActionSpace()
{
    uint32_t nActions = 3; // 0=유지, 1=gnb로 전환 2= rsu로 전환
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace>(nActions);
    return space;
}

void
MyExecuteActions(uint16_t choice)
{
    // Ptr<OpenGymDiscreteContainer> act = DynamicCast<OpenGymDiscreteContainer>(action);
    // uint32_t choice = act->GetValue();

    if (choice == 0)
    {
        std::cout << ">>> RL Agent chose KEEP (stay in current mode)" << std::endl;
    }
    else if (choice == 1)
    {
        std::cout << ">>> RL Agent chose SWITCH TO gNB" << std::endl;
        g_currentMode = 1;
        g_uuClientApps.Stop(Simulator::Now());                 // SL 끄고
        g_uuClientApps.Start(Simulator::Now() + Seconds(0.1)); // Uu 활성화
        g_switchCount++;
    }
    else if (choice == 2)
    {
        std::cout << ">>> RL Agent chose SWITCH TO RSU" << std::endl;
        g_currentMode = 2;
        g_uuClientApps.Stop(Simulator::Now()); // Uu 끄고
        Simulator::Schedule(Seconds(0.1), []() {
            Ptr<Packet> pkt = Create<Packet>(200);
            g_slSendSocket->SendTo(pkt, 0, g_remote);
        });
        g_switchCount++;
    }
    else
    {
        std::cout << ">>> RL Agent chose UNKNOWN ACTION" << std::endl;
    }

    // return true;
}

float
MyGetReward()
{
    Ptr<Node> ueNode = NodeList::GetNode(0);

    // 위치 기반 페널티/보상
    Vector pos = ueNode->GetObject<MobilityModel>()->GetPosition();
    float reward = 0.0;

    if (pos.x > 100.0)
    {
        reward += 1.0;
    }

    // 기존 Throughput, Loss, SINR 보상 계산 유지
    float throughput = 2.0;
    float packetLoss = 0.05;
    bool outage = false;

    reward += throughput;
    reward -= packetLoss * 10.0;
    if (outage)
        reward -= 100.0;

    reward += 0.1 * (g_lastRsrp + 100);
    reward += 0.1 * g_lastSinr;

    return reward;
}

bool
MyGetGameOver()
{
    return Simulator::Now().GetSeconds() >= 10.0;
}

// 단순 문자열 반환에서 JSON문자열로 바꿈, python에서 그대로 dict 변환이 가능 ?????? 이거 확인해
// 봐야 됨
std::string
MyGetExtraInfo()
{
    std::ostringstream oss;

    // 누적값을 기반으로 평균 throughput만 계산
    double simTime = Simulator::Now().GetSeconds();
    double avgThroughputUu = (simTime > 0) ? (g_totalRxBitsUu / simTime) / 1e6 : 0.0;

    oss << "{"
        << "\"avgThroughputUu\":" << avgThroughputUu << ", \"rxPacketsUu\":" << g_rxPacketsUu
        << ", \"switchCount\":" << g_switchCount << "}";

    return oss.str();
}

// 전역에 선언
static bool switched = false; // 한 번만 전환되도록 플래그

void
CheckPositionAndSwitch(Ptr<Node> ueNode,
                       double threshold,
                       Ptr<NrHelper> nrHelper,
                       Ptr<NrSlHelper> nrSlHelper,
                       Ptr<NetDevice> carDev,
                       Ptr<NetDevice> gnbDev,
                       Ptr<NetDevice> rsuDev,
                       SidelinkInfo slInfo,
                       uint16_t slPort,
                       Ipv4Address rsuAddress)
{
    if (switched)
        return;

    Vector pos = ueNode->GetObject<MobilityModel>()->GetPosition();

    if (pos.x > threshold && !switched)
    {
        switched = true;
        std::cout << ">>> UE" << ueNode->GetId() << " crossed threshold, switching to RSU(SL)"
                  << std::endl;

        g_uuClientApps.Stop(Simulator::Now());
    }
}

int
main(int argc, char* argv[])
{
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    /*
    명령줄에서 차량 수, 차선 수, 속도 등 입력값을 받을 수 있게 하는 파라미터
     */
    uint16_t numVehiclesPerLane = 5;
    uint16_t numLanes = 3;
    uint16_t interVehicleDist = 20; // meters
    uint16_t interLaneDist = 4;     // meters
    double speed = 38.88889;        // meter per second, default 140 km/h
    bool enableOneTxPerLane = true;
    bool logging = false;
    bool harqEnabled = true;
    Time delayBudget = Seconds(0); // Use T2 configuration

    // 트래픽 관련 파라미터 (패킷 크기, 데이터 속도)
    bool useIPv6 = false; // default IPV4
    uint32_t udpPacketSizeBe = 200;
    double dataRateBe = 16; // 16 kilobits per second

    // 시뮬레이션 관련 파라미터 (시뮬레이션 시간, 베어러 활성화 시점)
    Time simTime = Seconds(30);
    // 사이드링크 베어러가 켜지는 시점
    Time slBearersActivationTime = Seconds(2.0);

    // NR(5G) 통신 관련 파라미터 (명령줄 입력을 받아 NR 모듈에 넘김)
    double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
    double bandwidthBandSl = 40e6;          // 40 MHz
    uint32_t rbWidth = 12 * 30000;          // 30 kHz numerology → 360 kHz per RB
    uint16_t rbNum = static_cast<uint16_t>(bandwidthBandSl / rbWidth);
    std::cout << "DEBUG: bandwidthBandSl = " << bandwidthBandSl << " Hz, rbWidth = " << rbWidth
              << " Hz, rbNum = " << rbNum << std::endl;

    double txPower = 23; // dBm

    std::string tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";
    std::string slBitMap = "1|1|1|1|1|1|0|0|0|1|1|1";

    uint16_t numerologyBwpSl = 0;   // 원래 0에서 1로
    uint16_t slSensingWindow = 100; // T0 in ms
    uint16_t slSelectionWindow = 5; // T2min
    uint16_t slSubchannelSize = 10; // 50에서 10으로 줄임
    uint16_t slMaxNumPerReserve = 3;
    double slProbResourceKeep = 0.0;
    uint16_t slMaxTxTransNumPssch = 5; // Hz 단위
    uint16_t reservationPeriod = 100;  // in ms
    bool enableSensing = false;
    uint16_t t1 = 2;
    uint16_t t2 = 33;
    int slThresPsschRsrp = -128;
    bool enableChannelRandomness = false;
    uint16_t channelUpdatePeriod = 500; // ms
    uint8_t mcs = 14;
    uint16_t uuPort = 1234; // Uu (gNB-UE) 포트
    uint16_t slPort = 1235; // Sidelink (RSU-UE, UE-UE) 포트

    // gnuplot 스크립트를 만들기 여부를 설정하는 플래그
    bool generateInitialPosGnuScript = false;
    bool generateGifGnuScript = false;

    // 시뮬레이션 결과를 저장할 위치
    std::string simTag = "default";
    std::string outputDir = "./";
    bool saveDb = true;

    /*
     여기서부터는 CommandLine 클래스를 통해 명령줄에서 받을 수 있는 파라미터들을 정의하고 설명을
     붙이며, 저장할 변수와 연결함
     */
    CommandLine cmd(__FILE__);

    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("numVehiclesPerLane", "Number of vehicles per lane", numVehiclesPerLane);
    cmd.AddValue("numLanes", "Total Number of lanes going from West to East", numLanes);
    cmd.AddValue("interVehicleDist",
                 "inter-vehicle distance: it is the distance between the antenna position (located "
                 "in the center) of two vehicles in the same lane",
                 interVehicleDist);
    cmd.AddValue("interLaneDist",
                 "inter-lane distance: it is the distance between the antenna position (located in "
                 "the center) of two vehicles in the adjacent lane",
                 interLaneDist);
    cmd.AddValue("speed", "speed of the vehicles in m/sec", speed);
    cmd.AddValue("enableOneTxPerLane",
                 "Per lane make one vehicle a transmitter. This option only works"
                 "with odd number of UEs per lane, which makes the middle vehicle"
                 "in each lane a transmitter",
                 enableOneTxPerLane);
    cmd.AddValue("useIPv6", "Use IPv6 instead of IPv4", useIPv6);
    cmd.AddValue("packetSizeBe",
                 "packet size in bytes to be used by best effort traffic",
                 udpPacketSizeBe);
    cmd.AddValue("dataRateBe",
                 "The data rate in kilobits per second for best effort traffic",
                 dataRateBe);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("slBearerActivationTime",
                 "Sidelik bearer activation time in seconds",
                 slBearersActivationTime);
    cmd.AddValue("centralFrequencyBandSl",
                 "The central frequency to be used for Sidelink band/channel",
                 centralFrequencyBandSl);
    cmd.AddValue("bandwidthBandSl",
                 "The system bandwidth to be used for Sidelink",
                 bandwidthBandSl);
    cmd.AddValue("txPower", "total tx power in dBm", txPower);
    cmd.AddValue("tddPattern", "The TDD pattern string", tddPattern);
    cmd.AddValue("slBitMap", "The Sidelink bitmap string", slBitMap);
    cmd.AddValue("numerologyBwpSl",
                 "The numerology to be used in Sidelink bandwidth part",
                 numerologyBwpSl);
    cmd.AddValue("slSensingWindow", "The Sidelink sensing window length in ms", slSensingWindow);
    cmd.AddValue("slSelectionWindow",
                 "The parameter which decides the minimum Sidelink selection "
                 "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                 slSelectionWindow);
    cmd.AddValue("slSubchannelSize", "The Sidelink subchannel size in RBs", slSubchannelSize);
    cmd.AddValue("slMaxNumPerReserve",
                 "The parameter which indicates the maximum number of reserved "
                 "PSCCH/PSSCH resources that can be indicated by an SCI.",
                 slMaxNumPerReserve);
    cmd.AddValue("slProbResourceKeep",
                 "The parameter which indicates the probability with which the "
                 "UE keeps the current resource when the resource reselection"
                 "counter reaches zero.",
                 slProbResourceKeep);
    cmd.AddValue("slMaxTxTransNumPssch",
                 "The parameter which indicates the maximum transmission number "
                 "(including new transmission and retransmission) for PSSCH.",
                 slMaxTxTransNumPssch);
    cmd.AddValue("enableSensing",
                 "If true, it enables the sensing based resource selection for "
                 "SL, otherwise, no sensing is applied",
                 enableSensing);
    cmd.AddValue("t1",
                 "The start of the selection window in physical slots, "
                 "accounting for physical layer processing delay",
                 t1);
    cmd.AddValue("t2", "The end of the selection window in physical slots", t2);
    cmd.AddValue("slThresPsschRsrp",
                 "A threshold in dBm used for sensing based UE autonomous resource selection",
                 slThresPsschRsrp);
    cmd.AddValue("enableChannelRandomness",
                 "Enable shadowing and channel updates",
                 enableChannelRandomness);
    cmd.AddValue("channelUpdatePeriod", "The channel update period in ms", channelUpdatePeriod);
    cmd.AddValue("mcs", "The MCS to used for sidelink", mcs);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("simTag", "tag identifying the simulation campaigns", simTag);
    cmd.AddValue("saveDb", "Flag to control the saving of database file", saveDb);
    cmd.AddValue("generateInitialPosGnuScript",
                 "generate gnuplot script to plot initial positions of the UEs",
                 generateInitialPosGnuScript);
    cmd.AddValue("generateGifGnuScript",
                 "generate gnuplot script to generate GIF to show UEs mobility",
                 generateGifGnuScript);

    cmd.Parse(argc, argv);

    /*
    주파수가 허용된 범위 안에 있는지 확인, 다른 검증이 필요하면 이 위치에서 하면 됨
     */
    NS_ABORT_IF(centralFrequencyBandSl > 6e9);

    /*
    logging이 true라면 특정 컴포넌트의 로그를 켬.
    같은 효과는 환경 변수 NS_LOG 설정으로도 가능함
     */
    if (logging)
    {
        // DEBUG/ALL 레벨까지 찍게 변경
        LogLevel logLevel = (LogLevel)(LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);

        // OnOffApplication (클라이언트 송신)
        LogComponentEnable("OnOffApplication", logLevel);

        // PacketSink (서버 수신)
        LogComponentEnable("PacketSink", logLevel);
    }

    /*
     시뮬레이션 기본값 설정
     */
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::NrGnbMac::NumRbPerRbg", UintegerValue(1));

    /*
     모든 UE(차량 단말)를 담는 NodeContainer 생성
     */
    NodeContainer allSlUesContainer;

    /*
     * 차량 (UE)에 이동 모델 할당
     *  1. 이동 모델 종류 설정
     *  2. 차량 위치 지정
     *  3. 이동 모델 적용
     */
    NS_ABORT_MSG_IF(
        numVehiclesPerLane % 2 == 0 && enableOneTxPerLane == true,
        "We only support odd number of vehicles per lane if enableOneTxPerLane is true");
    allSlUesContainer = InstallHighwayMobility(numLanes,
                                               numVehiclesPerLane,
                                               interVehicleDist,
                                               interLaneDist,
                                               speed);

    /*
     NR 모듈 설정, 시뮬레이션에 필요한 헬퍼 객체 생성:
     - EPCHepler : 코어 네트워크 설정
     - NrHelper : NR 스택의 구성요소를 생성하고 연결
     */
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetEpcHelper(epcHelper);

    // NrSlHelper 생성
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);

    // SidelinkInfo 준비
    uint32_t dstL2Id = 255;
    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(reservationPeriod);
    slInfo.m_dynamic = false;
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = harqEnabled;

    /*  주파수 대역 설정, 입력된 중심 주파수로 한 개의 대역과 대역폭 파트를 만들고, StreetCanyon
     * 채널 모델을 사용  */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    // 수정
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl, // Hz 단위
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);

    // By using the configuration created, it is time to make the operation bands
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);

    /*
     * 주파수 대역 구성
     * ------------Band1--------------
     * ------------CC1----------------
     * ------------BwpSl--------------
     */
    if (enableChannelRandomness)
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                           TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                    TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    }
    else
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    }

    /* 채널과 경로손실 등을 초기화 → 여기선 자동 초기화 사용   */
    nrHelper->InitializeOperationBand(&bandSl);
    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    NodeContainer gNb;
    gNb.Create(1);
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gNb);

    // gnb 장치 설치
    NetDeviceContainer gNbDevice = nrHelper->InstallGnbDevice(gNb, allBwps);

    // === 여기서 추가 ===
    for (auto it = gNbDevice.Begin(); it != gNbDevice.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(*it);
        if (gnbDev)
        {
            gnbDev->UpdateConfig(); // 여기서 설정 반영됨
            std::cout << "===== DEBUG CHECK =====" << std::endl;
            std::cout << "Num RBs = " << gnbDev->GetPhy(0)->GetRbNum() << std::endl;
            std::cout << "Num RB per RBG = " << gnbDev->GetMac(0)->GetNumRbPerRbg() << std::endl;
            std::cout << "=======================" << std::endl;
        }
    }

    /*  모든 차량(UE) 안테나 설정
     SL에서는 빔포밍 대신 준-전방위 ?? 송수신을 사용 (기본값)   */
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));

    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(enableSensing));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(static_cast<uint8_t>(t1)));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(t2));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
    nrHelper->SetUeMacAttribute("SlThresPsschRsrp", IntegerValue(slThresPsschRsrp));

    uint8_t bwpIdForGbrMcptt = 0;

    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));

    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    // (추가) UE에 InternetStack 설치 (Attach 전에 반드시 필요!)
    InternetStackHelper internet;
    internet.Install(allSlUesContainer);

    // ue 장치 설치
    NetDeviceContainer allSlUesNetDeviceContainer =
        nrHelper->InstallUeDevice(allSlUesContainer, allBwps);

    // === 여기 이후에 Trace 연결해야 함 ===
    Ptr<NetDevice> rsuDev = allSlUesNetDeviceContainer.Get(0); // RSU 디바이스
    rsuDev->TraceConnectWithoutContext("PhyRxEnd", MakeCallback(&RxTraceCallback));

    // --- IP 할당 ---
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(allSlUesNetDeviceContainer);

    // RSU (UE0) 주소 확보
    Ipv4Address rsuAddress = ueIpIface.GetAddress(0);
    std::cout << ">>> RSU(UE0) has IP = " << rsuAddress << std::endl;

    // === UE1 socket 생성 ===
    // === UE1 socket 생성 ===
    Ptr<Socket> ue1Socket =
        Socket::CreateSocket(allSlUesContainer.Get(1), UdpSocketFactory::GetTypeId());
    ue1Socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 0));
    std::cout << ">>> DEBUG: UE1 socket created (no connect)" << std::endl;

    // 패킷 송신
    Simulator::Schedule(Seconds(0.5), &SendSlPacket, ue1Socket, rsuAddress, 1235);
    Simulator::Schedule(Seconds(1.0), &SendSlPacket, ue1Socket, rsuAddress, 1235);

    // --- UE1 to RSU routing ---
    Ptr<Ipv4> ue1Ipv4 = allSlUesContainer.Get(1)->GetObject<Ipv4>();

    // 라우팅 설정 (RSU IP로 가는 패킷은 IfIndex=1로 보냄)
    Ptr<Ipv4StaticRouting> ue1StaticRouting = ipv4RoutingHelper.GetStaticRouting(ue1Ipv4);
    ue1StaticRouting->AddHostRouteTo(rsuAddress, 1);

    std::cout << ">>> DEBUG: Forced route UE1 -> RSU(" << rsuAddress << ") via IfIndex=1"
              << std::endl;

    // --- DEBUG: print UE1 routing table ---
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(&std::cout);
    ue1Ipv4->GetRoutingProtocol()->PrintRoutingTable(routingStream);

    // 디버그용: UE1 인터페이스 정보 출력
    for (uint32_t i = 0; i < ue1Ipv4->GetNInterfaces(); i++)
    {
        for (uint32_t j = 0; j < ue1Ipv4->GetNAddresses(i); j++)
        {
            Ipv4InterfaceAddress iaddr = ue1Ipv4->GetAddress(i, j);
            std::cout << "UE1 IfIndex=" << i << " IP=" << iaddr.GetLocal() << std::endl;
        }
    }

    // When all the configuration is done, explicitly call UpdateConfig ()
    for (auto it = allSlUesNetDeviceContainer.Begin(); it != allSlUesNetDeviceContainer.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // 이 위치로 옮겨야 정상 동작
    Ptr<Node> ueNode = allSlUesContainer.Get(1); // UE1(Car) 노드 선언

    // EPC ↔ Remote Host 연결
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // 원격 서버 노드 생성
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // 인터넷 스택 설치
    InternetStackHelper internetRemote;
    internetRemote.Install(remoteHostContainer);

    // PGW ↔ Remote Host 연결 (PointToPoint 링크)
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(reservationPeriod);
    slInfo.m_dynamic = false;
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = harqEnabled;

    /*
    사이드링크(V2X) 설정을 위해 헬퍼 객체 생성:
    - NrSlHelper : 차량 단말 스택을 SL용으로 구성
    - EpcHelper : 베어러 설정 절차 담당
    */

    nrSlHelper->SetEpcHelper(epcHelper);

    /*
     SL 오류 모델과 AMC 설정
     - 오류 모델 : NrEesmCct1 등
     - AMC 모델 : ShannonModel 또는 ErrorModel
     */
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    /*
     SL 스케줄러 속성 설정 - 여기서는 단일 MCS를 사용하는 간단한 스케줄러 사용
     */
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(mcs));

    /*
    UE 프로토콜 스택을 실제를 구성하는 중요한 단계
    → 계층 간 인터페이스(SAP), 콜백, 오류 모델, AMC, 간섭 API 등을 설정함
     */
    nrSlHelper->PrepareUeForSidelink(allSlUesNetDeviceContainer, bwpIdContainer);

    /*
     사이드링크 사전 설정 구조체 준비 시작 → 이 구조체는 SL 관련 모든 사전 설정 정보를 담음
     */

    // SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();

    std::vector<std::bitset<1>> slBitMapVector;
    GetSlBitmapFromString(slBitMap, slBitMapVector);
    NS_ABORT_MSG_IF(slBitMapVector.empty(), "GetSlBitmapFromString failed to generate SL bitmap");
    ptrFactory->SetSlTimeResources(slBitMapVector);
    ptrFactory->SetSlSensingWindow(slSensingWindow); // T0 in ms
    ptrFactory->SetSlSelectionWindow(slSelectionWindow);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(slSubchannelSize);
    ptrFactory->SetSlMaxNumPerReserve(slMaxNumPerReserve);
    std::list<uint16_t> resourceReservePeriodList = {0, reservationPeriod}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, we can create the pool
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    // Pool id, ranges from 0 to 15
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    // Array for pools, we insert the pool in the array as per its poolId
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    // bwp.rbPerRbg = 1;
    bwp.bandwidth = rbNum; // Hz 단위가 아니라 RB 개수!

    std::cout << "DEBUG: bwp.bandwidth set to " << bwp.bandwidth << " RBs" << std::endl;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;

    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = tddPattern;

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    NS_ABORT_MSG_UNLESS(slProbResourceKeep <= 1.0,
                        "slProbResourceKeep value must be between 0 and 1");
    slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = static_cast<uint8_t>(slMaxTxTransNumPssch);
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(allSlUesNetDeviceContainer, slPreConfigNr);

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(allSlUesNetDeviceContainer, stream);
    stream += nrSlHelper->AssignStreams(allSlUesNetDeviceContainer, stream);

    NodeContainer txSlUes;
    NodeContainer rxSlUes;
    NetDeviceContainer txSlUesNetDevice;
    NetDeviceContainer rxSlUesNetDevice;
    if (enableOneTxPerLane)
    {
        for (uint16_t i = 1; i <= numLanes * numVehiclesPerLane; i++)
        {
            // for each lane one transmitter
            if (i % numVehiclesPerLane == 0)
            {
                uint16_t firstIndex = (i - numVehiclesPerLane) + 1;
                uint16_t txNodeId = (firstIndex + i) / 2;
                txNodeId = txNodeId - 1; // node id starts from 0
                txSlUes.Add(allSlUesContainer.Get(txNodeId));

                Ptr<NrUeNetDevice> ueDev = GetUeNetDevice(allSlUesContainer.Get(txNodeId));
                NS_ABORT_MSG_IF(ueDev == nullptr, "No NrUeNetDevice found on this node");

                txSlUesNetDevice.Add(ueDev);
            }
        }
        // all node ids, which are not in txSlUes container are Rx node ids
        for (uint32_t i = 0; i < allSlUesContainer.GetN(); ++i)
        {
            Ptr<Node> node = allSlUesContainer.Get(i);

            Ptr<NrUeNetDevice> ueDev = GetUeNetDevice(node); // 선언 + 초기화
            NS_ABORT_MSG_IF(ueDev == nullptr, "No NrUeNetDevice found on this node");

            rxSlUesNetDevice.Add(ueDev);
        }
    }

    else
    {
        txSlUes.Add(allSlUesContainer);
        rxSlUes.Add(allSlUesContainer);
        txSlUesNetDevice.Add(allSlUesNetDeviceContainer);
        rxSlUesNetDevice.Add(allSlUesNetDeviceContainer);
    }

    // 차량(UE1~UE14)만 attach
    NetDeviceContainer ueCars;
    for (uint32_t i = 1; i < allSlUesNetDeviceContainer.GetN(); ++i)
    {
        ueCars.Add(allSlUesNetDeviceContainer.Get(i));
    }

    // (2) SL bearer는 RSU(UE0) ↔ 차량(UE1)만 활성화
    Ptr<LteSlTft> tftTx =
        Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, Ipv4Address("225.0.0.0"), slPort, slInfo);

    Ptr<LteSlTft> tftRx =
        Create<LteSlTft>(LteSlTft::Direction::RECEIVE, Ipv4Address("225.0.0.0"), slPort, slInfo);

    NetDeviceContainer txDev, rxDev;
    txDev.Add(allSlUesNetDeviceContainer.Get(1)); // UE1 (Car)
    rxDev.Add(allSlUesNetDeviceContainer.Get(0)); // UE0 (RSU)

    // --- SL Server (RSU = UE0) ---
    Ptr<Node> rsuNode = allSlUesContainer.Get(0); // UE0 = RSU
    Ptr<Socket> slRecvSocket = Socket::CreateSocket(rsuNode, UdpSocketFactory::GetTypeId());

    // RSU의 실제 IP로 바인딩 (예: 7.0.0.2:slPort)
    InetSocketAddress local = InetSocketAddress(rsuAddress, slPort);
    int bindResult = slRecvSocket->Bind(local);

    if (bindResult == 0)
    {
        slRecvSocket->SetRecvCallback(MakeCallback(&NotifySlRx));
        std::cout << ">>> RSU socket bound SUCCESS at " << rsuAddress << ":" << slPort << std::endl;
    }
    else
    {
        std::cout << ">>> ERROR: RSU socket bind FAILED at " << rsuAddress << ":" << slPort
                  << " (code=" << bindResult << ")" << std::endl;
    }

    // --- SL Client (Car = UE1) ---
    Ptr<Node> carNode = allSlUesContainer.Get(1); // UE1(Car)

    // UE1 장치 확인
    Ptr<NrUeNetDevice> ueDev = GetUeNetDevice(carNode);
    NS_ABORT_MSG_IF(ueDev == nullptr, "No NrUeNetDevice found on UE1");

    // 송신 소켓 생성
    Ptr<Socket> slSendSocket = Socket::CreateSocket(carNode, UdpSocketFactory::GetTypeId());

    // 송신 소켓 바인딩 → 동적 포트 사용 (0 지정)
    slSendSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 0)); // 포트 충돌 방지

    // RSU 주소를 remote 로 명시
    InetSocketAddress remote = InetSocketAddress(rsuAddress, slPort);
    slSendSocket->Connect(remote);

    // UE1 -> RSU 패킷 송신 (함수 호출)
    Simulator::Schedule(Seconds(0.5), &SendSlPacket, slSendSocket, rsuAddress, slPort);
    Simulator::Schedule(Seconds(1.0), &SendSlPacket, slSendSocket, rsuAddress, slPort);

    Simulator::Schedule(Seconds(1.0),
                        &CheckPositionAndSwitch,
                        allSlUesContainer.Get(1),
                        100.0,
                        nrHelper,
                        nrSlHelper,
                        allSlUesNetDeviceContainer.Get(1), // carDev
                        gNbDevice.Get(0),                  // gnbDev ← 빠졌던 부분!
                        allSlUesNetDeviceContainer.Get(0), // rsuDev
                        slInfo,
                        slPort,
                        rsuAddress);

    stream += internet.AssignStreams(allSlUesContainer, stream);
    // uint32_t dstL2Id = 255;
    Ipv4Address groupAddress4("225.1.0.0"); // use multicast address as destination
    Ipv6Address groupAddress6("ff0e::1");   // use multicast address as destination
    Address remoteAddress;
    Address localAddress;
    Ptr<LteSlTft> sltft;
    // SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(reservationPeriod);
    slInfo.m_dynamic = false;
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = harqEnabled;

    if (!useIPv6)
    {
        // Ipv4StaticRoutingHelper ipv4RoutingHelper;
        for (uint32_t u = 0; u < allSlUesContainer.GetN(); ++u)
        {
            Ptr<Node> ueNode = allSlUesContainer.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
        remoteAddress = InetSocketAddress(groupAddress4, slPort);
        localAddress = InetSocketAddress(Ipv4Address::GetAny(), slPort);
    }

    // === Uu Application 추가 (IPv4/IPv6 상관없이 항상 실행) ===
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());

    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                               Ipv4Mask("255.0.0.0"),
                                               internetDevices.Get(0)->GetIfIndex());

    // === UE IP 할당 확인용 디버깅 로그 추가 ===
    for (uint32_t u = 0; u < ueIpIface.GetN(); ++u)
    {
        std::cout << ">>> UE" << u << " got IP = " << ueIpIface.GetAddress(u) << std::endl;
    }

    // --- Uu Server (RemoteHost 수신) ---
    // uint16_t uuPort = 5000;   // Uu 포트 (Client/Server 모두 동일하게!)
    ApplicationContainer uuServerApps;

    PacketSinkHelper uuSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), uuPort));

    uuSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    uuServerApps = uuSink.Install(remoteHost);

    uuServerApps.Start(Seconds(0.1));
    uuServerApps.Stop(simTime);

    // === Rx 콜백 연결 (Throughput 집계용) ===
    if (uuServerApps.GetN() > 0)
    {
        Ptr<PacketSink> uuSinkApp = DynamicCast<PacketSink>(uuServerApps.Get(0));
        if (uuSinkApp)
        {
            std::cout << ">>> DEBUG: Uu Rx trace connected" << std::endl;
            uuSinkApp->TraceConnectWithoutContext("Rx", MakeCallback(&NotifyUuRx));
        }
    }

    // --- Uu Client (UE1 송신) ---
    OnOffHelper uuClientHelper("ns3::UdpSocketFactory", InetSocketAddress(remoteHostAddr, uuPort));
    uuClientHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    uuClientHelper.SetConstantRate(DataRate("1Mbps"), 200);
    uuClientHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    uuClientHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    // 전역 변수에 저장
    g_uuClientApps = uuClientHelper.Install(allSlUesContainer.Get(1));
    g_uuClientApps.Start(Seconds(1.0));

    // --- SL 소켓 생성 → 전역 변수에 저장 ---
    g_slSendSocket = Socket::CreateSocket(allSlUesContainer.Get(1),
                                          TypeId::LookupByName("ns3::UdpSocketFactory"));

    g_slSendSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 0));
    g_remote = InetSocketAddress(rsuAddress, slPort);
    g_slSendSocket->Connect(g_remote);

    // --- 라우팅 추가 (RemoteHost → UEs) ---
    // Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
    // ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());

    /*
     * Hook the traces, for trace data to be stored in a database
     */
    std::string exampleName = simTag + "-" + "nr-v2x-west-to-east-highway";

    // DB를 new로 생성
    g_db = std::make_unique<SQLiteOutput>(outputDir + exampleName + ".db");

    // Stats와 DB 연결
    g_rlcRxStats.SetDb(g_db.get(), "rlcRx");

    // Trace 연결
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/RxRlcPduWithTxRnti",
                                  MakeBoundCallback(&NotifySlRlcPduRx, &g_rlcRxStats));

    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/RxRlcPduWithTxRnti",
                                  MakeCallback(&NotifyUuRlcPduRx));

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                    "ComponentCarrierMapUe/*/NrUePhy/ReportRsrp",
                    MakeCallback(&RsrpCallback));

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                    "ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                    MakeCallback(&SinrCallback));

    // OpenGym Environment 초기화
    // uint32_t openGymPort = 5555;
    // Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface>(openGymPort);
    //
    // // 콜백 함수 등록
    // openGymInterface->SetGetObservationSpaceCb(MakeCallback(&MyGetObservationSpace));
    // openGymInterface->SetGetObservationCb(MakeCallback(&MyGetObservation));
    // openGymInterface->SetGetActionSpaceCb(MakeCallback(&MyGetActionSpace));
    // openGymInterface->SetExecuteActionsCb(MakeCallback(&MyExecuteActions));
    // openGymInterface->SetGetRewardCb(MakeCallback(&MyGetReward));
    // openGymInterface->SetGetGameOverCb(MakeCallback(&MyGetGameOver));
    // openGymInterface->SetGetExtraInfoCb(MakeCallback(&MyGetExtraInfo));

    // Step마다 Observation 전달 예약
    // Time stepTime = Seconds(1.0);
    // Simulator::Schedule(stepTime, &ScheduleNextStateRead, stepTime, openGymInterface);
    //
    // std::cout << "=== OpenGymInterface started on port " << openGymPort << " ===" << std::endl;
    {
        float f = static_cast<float>(0.5);
        Simulator::Schedule(Seconds(f), []() {
            Ptr<Packet> pkt = Create<Packet>(200);
            g_slSendSocket->SendTo(pkt, 0, g_remote);
        });
        Simulator::Schedule(Seconds(f), &SendSlPacket, ue1Socket, rsuAddress, 1235);
        Simulator::Schedule(Seconds(f), &MyExecuteActions, 1);
    }

    Simulator::Schedule(Seconds(0.0), &PrintUeInfo, ueNode);
    for (uint16_t i = 1; i < 30; i++)
    {
        float f = static_cast<float>(i);
        Simulator::Schedule(Seconds(f), []() {
            Ptr<Packet> pkt = Create<Packet>(200);
            g_slSendSocket->SendTo(pkt, 0, g_remote);
        });
        Simulator::Schedule(Seconds(f), &SendSlPacket, ue1Socket, rsuAddress, 1235);
        Simulator::Schedule(Seconds(f), &MyExecuteActions, 2);
    }

    Config::ConnectWithoutContext("/NodeList/" + std::to_string(rsuNode->GetId()) +
                                      "/$ns3::Ipv4L3Protocol/Rx",
                                  MakeCallback(&Ipv4PacketTraceAtRsu));
    // ---- 이제 딱 한 번만 실행 ----
    Simulator::Stop(simTime);
    Simulator::Run();

    // --- 수신 기반 스루풋 계산 ---
    double totalTime = Simulator::Now().GetSeconds();
    double throughputMbps = (g_totalRxBitsSl / 1e6) / totalTime;

    std::cout << ">>> Total SL Rx Packets = " << g_rxPacketsSl << std::endl;
    std::cout << ">>> Total SL Rx Bits = " << g_totalRxBitsSl << " bits" << std::endl;
    std::cout << ">>> Average SL Throughput = " << throughputMbps << " Mbps" << std::endl;

    Simulator::Destroy();

    std::cout << "=== NS-3 Simulation finished successfully ===" << std::endl;

    if (!saveDb)
    {
        std::remove((outputDir + exampleName + ".db").c_str());
    }

    return 0;
}