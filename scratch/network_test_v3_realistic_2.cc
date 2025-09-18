/*
 * network test 1ue 1rsu 1gnb 1router 1server
 *
 *      gnb - pgw
 *    /           \
 *  ue            router ----server
 *   \              /
 *     rsu --------
*/

#include "ns3/address.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/csma-helper.h"
#include "ns3/epc-tft.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-sl-ue-cphy-sap.h"
#include "ns3/oh-buildings-propagation-loss-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/udp-koh-client.h"
#include "ns3/udp-koh-server.h"
#include "ns3/quic-koh-server.h"
#include "ns3/quic-koh-client.h"
#include <ns3/pointer.h>
using namespace ns3;

class StackHelper // 클래스 이름 변경
{
public:
    // IPv4 라우팅 테이블 출력 함수
    inline void PrintRoutingTable(Ptr<Node>& n)
    {
        // 모든 클래스를 Ipv4용으로 변경
        Ptr<Ipv4StaticRouting> routing = nullptr;
        Ipv4StaticRoutingHelper routingHelper;
        Ptr<Ipv4> ipv4 = n->GetObject<Ipv4>();
        uint32_t nbRoutes = 0;
        Ipv4RoutingTableEntry route;

        routing = routingHelper.GetStaticRouting(ipv4);
        if (!routing)
        {
            std::cout << "Node " << n->GetId() << " has no static routing installed." << std::endl;
            return;
        }

        std::cout << "--- IPv4 Routing table of Node " << n->GetId() << " ---" << std::endl;
        std::cout << "Destination\tMask\t\tGateway\t\tInterface" << std::endl;

        nbRoutes = routing->GetNRoutes();
        for (uint32_t i = 0; i < nbRoutes; i++)
        {
            route = routing->GetRoute(i);
            std::cout << route.GetDest() << "\t"
                      << route.GetDestNetworkMask() << "\t"
                      << route.GetGateway() << "\t"
                      << route.GetInterface()
                      << std::endl;
        }
        std::cout << "------------------------------------" << std::endl;
    }
};


struct WaypointData
{
    double time;
    double x;
    double y;
    double z;
    double speed;
};

void UeMeasCallback(uint16_t cellId, uint16_t IMSI, uint16_t RNTI, double RSRP, uint8_t BWPId)
{
    std::cout << "📶Uu [Meas] cellId=" << cellId << " IMSI=" << IMSI << " BWPId=" << BWPId
              << "  RNTI=" << RNTI << " RSRP=" << RSRP << " dB\n";
}

void UeSlMeasCallback(uint16_t RNTI, uint32_t L2ID, double RSRP)
{
    std::cout << "📶Sl [Meas] RNTI=" << RNTI << " L2ID=" << L2ID << " RSRP=" << RSRP << " dB\n";
}

#include <ns3/spectrum-model.h>
#include <ns3/spectrum-value.h>

#include <cmath>

#include "ns3/spectrum-phy.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/net-device.h"
#include "ns3/node.h"



// UE의 위치와 속도를 출력하는 함수
// void PrintUeInfo(Ptr<Node> ueNode1, Ptr<Node> ueNode2)
// {
//     Ptr<MobilityModel> mob1 = ueNode1->GetObject<MobilityModel>();
//     Vector pos1 = mob1->GetPosition();
//     Vector vel1 = mob1->GetVelocity();
//
//     Ptr<MobilityModel> mob2 = ueNode2->GetObject<MobilityModel>();
//     Vector pos2 = mob2->GetPosition();
//     Vector vel2 = mob2->GetVelocity();
//
//     NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
//     NS_LOG_UNCOND("Vehicle 1");
//     NS_LOG_UNCOND("UE Position: x=" << pos1.x << ", y=" << pos1.y << ", z=" << pos1.z);
//     NS_LOG_UNCOND("UE Velocity: x=" << vel1.x << ", y=" << vel1.y << ", z=" << vel1.z << " (m/s)");
//     NS_LOG_UNCOND("Vehicle 2");
//     NS_LOG_UNCOND("UE Position: x=" << pos2.x << ", y=" << pos2.y << ", z=" << pos2.z);
//     NS_LOG_UNCOND("UE Velocity: x=" << vel2.x << ", y=" << vel2.y << ", z=" << vel2.z << " (m/s)");
//     Simulator::Schedule(Seconds(1.0), &PrintUeInfo, ueNode1, ueNode2);
// }

void PrintUeInfo(NodeContainer ueNodes)
{
    uint8_t i=0;
    NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
    for (auto it= ueNodes.Begin(); it!=ueNodes.End();++it)
    {
        Ptr<Node> ueNode = *it;
        Ptr<MobilityModel> mob = ueNode->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector vel = mob->GetVelocity();
        NS_LOG_UNCOND("Vehicle "<<++i);
        NS_LOG_UNCOND("UE Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z);
        NS_LOG_UNCOND("UE Velocity: x=" << vel.x << ", y=" << vel.y << ", z=" << vel.z << " (m/s)");
    }
    Simulator::Schedule(Seconds(1.0), &PrintUeInfo, ueNodes);
}

// 패킷 정보를 출력할 콜백 함수
void Ipv4PacketTraceAtRsu(Ptr<const Packet> packet, Ptr<Ipv4> Ipv4, uint32_t interfaceIndex)
{
    Ipv4Header Ipv4Header;
    packet->PeekHeader(Ipv4Header);

    std::cout << "[RSU Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

void Ipv4PacketTraceAtPgw(Ptr<const Packet> packet, Ptr<Ipv4> Ipv4, uint32_t interfaceIndex)
{
    Ipv4Header Ipv4Header;
    packet->PeekHeader(Ipv4Header);

    std::cout << "[PGW Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

void Ipv4PacketTraceAtRouter(Ptr<const Packet> packet, Ptr<Ipv4> Ipv4, uint32_t interfaceIndex)
{
    Ipv4Header Ipv4Header;
    packet->PeekHeader(Ipv4Header);

    std::cout << "[Router Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

void Ipv4PacketTraceAtServer(Ptr<const Packet> packet, Ptr<Ipv4> Ipv4, uint32_t interfaceIndex)
{
    Ipv4Header Ipv4Header;
    packet->PeekHeader(Ipv4Header);

    std::cout << "[Server Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

void Ipv4PacketTraceAtUe(Ptr<const Packet> packet, Ptr<Ipv4> Ipv4, uint32_t interfaceIndex)
{
    Ipv4Header Ipv4Header;
    packet->PeekHeader(Ipv4Header);

    std::cout << "[Ue Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}

int main(void)
{
    Ipv4Address groupAddress4("224.1.1.1");
    uint16_t ueNum = 2;
    Time simTime = Seconds(93);
    std::string csvFileName = "/home/kiho/ns-3-quic/scratch/final_3d_trace";

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    NodeContainer gnbNodeContainer;
    gnbNodeContainer.Create(1);
    NodeContainer rsuNodeContainer;
    rsuNodeContainer.Create(1);
    NodeContainer serverNodeContainer;
    serverNodeContainer.Create(1);
    NodeContainer ueNodeContainer;
    ueNodeContainer.Create(ueNum);
    NodeContainer routerNodeContainer;
    routerNodeContainer.Create(1);

    Ptr<Node> pgw = epcHelper->GetPgwNode(); // ipv4, Ipv4 둘다 설치되어 있음. 듀얼스택
    Ptr<Node> server = serverNodeContainer.Get(0);
    Ptr<Node> rsu = rsuNodeContainer.Get(0);
    Ptr<Node> gnb = gnbNodeContainer.Get(0);
    Ptr<Node> router = routerNodeContainer.Get(0);
    Ptr<Node> ue1 = ueNodeContainer.Get(0);
    Ptr<Node> ue2 = ueNodeContainer.Get(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodeContainer);
    gnbNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(900, 4288, 100.0));
    mobility.Install(rsuNodeContainer);
    rsuNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(294.0, 4350.0, 70.0));
    mobility.Install(serverNodeContainer);
    serverNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(1900.0, 3800.0, 60.0));
    mobility.Install(routerNodeContainer);
    routerNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(900.0,4000.0,80.0));

    //UE위치 할당
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(ueNodeContainer);
    for (uint32_t i = 0; i < ueNum; i++)
    {
        std::vector<WaypointData> waypoints;
        std::string finalFileName;
        if (i==0)
        {
            finalFileName = csvFileName+".csv";
        }else
        {
            finalFileName = csvFileName+"_"+std::to_string(i)+".csv";
        }

        ueNodeContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(294.0, 4355.03, 59));
        Ptr<WaypointMobilityModel> ueMobility = ueNodeContainer.Get(i)->GetObject<WaypointMobilityModel>();


        std::ifstream file(finalFileName);

        if (!file.is_open())
        {
            std::cout << "Could not open CSV file: " << finalFileName << std::endl;
            return 2;
        }

        std::string line;
        // 헤더 라인 무시
        std::getline(file, line);

        double maxTime = 0.0;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string value;
            WaypointData data;

            // 각 열 파싱
            std::getline(ss, value, ','); // time
            data.time = std::stod(value);
            std::getline(ss, value, ','); // vehicle_id (skip)
            std::getline(ss, value, ','); // x
            data.x = std::stod(value);
            std::getline(ss, value, ','); // y
            data.y = std::stod(value);
            std::getline(ss, value, ','); // z
            data.z = std::stod(value);
            std::getline(ss, value, ','); // speed
            data.speed = std::stod(value);
            std::getline(ss, value, ','); // lon (skip)
            std::getline(ss, value, ','); // lat (skip)

            waypoints.push_back(data);
            if (data.time > maxTime)
            {
                maxTime = data.time;
            }
        }
        file.close();
        NS_LOG_UNCOND("Successfully read " << waypoints.size() << " waypoints from CSV.");

        // 읽어온 CSV 데이터를 Waypoint로 추가
        for (const auto& data : waypoints)
        {
            Waypoint waypoint(Seconds(data.time), Vector(data.x, data.y, data.z));
            ueMobility->AddWaypoint(waypoint);
        }
    }



    // gnb bwp 설정

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    double gNbFrequencyBand = 3.5e9; // 3.5GHz
    double gNbBandwidthBand = 1e8;   // 100MHz
    uint8_t gNbnumContiguousCc = 1;  // 100MHz 안에 몇개의 CC가 들어가 있는지
    uint16_t gNbNumerology = 1;
    double gNbTxPower = 43.0; // 단위dBm

    CcBwpCreator gNbCcBwpCreators;
    OperationBandInfo gNbBand;

    CcBwpCreator::SimpleOperationBandConf gNbBandConf(gNbFrequencyBand,
                                                      gNbBandwidthBand,
                                                      gNbnumContiguousCc,
                                                      BandwidthPartInfo::RMa_LoS); //
    gNbBandConf.m_numBwp = 1;                                                                    // 1 BWP per CC
    gNbBand = gNbCcBwpCreators.CreateOperationBandContiguousCc(gNbBandConf);

    nrHelper->InitializeOperationBand(&gNbBand);
    BandwidthPartInfoPtrVector gNbBwp;
    gNbBwp = CcBwpCreator::GetAllBwps({gNbBand});

    std::vector<ObjectFactory> macUuFactory;
    ObjectFactory uufactory;
    uufactory.SetTypeId(NrUeMac::GetTypeId());
    macUuFactory.push_back(uufactory);
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodeContainer, gNbBwp);

    // Ptr<OhBuildingsPropagationLossModel> bldgLoss =
    // CreateObject<OhBuildingsPropagationLossModel>();
    //
    // bldgLoss->SetAttribute("ShadowSigmaOutdoor", DoubleValue(6.0));
    // bldgLoss->SetAttribute("ShadowSigmaIndoor", DoubleValue(7.0));
    // bldgLoss->SetAttribute("ShadowSigmaExtWalls", DoubleValue(5.0));
    // bldgLoss->SetAttribute("InternalWallLoss", DoubleValue(5.0));
    //
    // // NRHelper에 적용
    // nrHelper->SetPathlossAttribute("PathlossModel", PointerValue(bldgLoss));

    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    // 안테나 설정
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_PREMIUM",
                                                 UintegerValue(0)); // bwp하나만 한거

    std::string pattern = "DL|DL|DL|DL|UL|UL|UL|UL|UL|UL|";
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(gNbNumerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(gNbTxPower));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Pattern", StringValue(pattern));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetNoiseFigure(5.0);

    // 설정 적용
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    // ue uu 설정
    NetDeviceContainer ueUuNetDev =
        nrHelper->InstallUeDevice(ueNodeContainer, gNbBwp, macUuFactory);

    //노이즈 설정 부분
    nrHelper->GetUePhy(ueUuNetDev.Get(0), 0)->SetNoiseFigure(7.0);

    double ueTxPower = 23.0;
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->GetUePhy(ueUuNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(ueTxPower));


    DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0))->UpdateConfig();

    // RSU, SL 기본 설정=======================================================
    double RsuFrequencyBand = 5.89e9;
    double   RsuBandwidthHz   = 20e6;
    // uint16_t RsuBandwidthBand = 400;
    // uint16_t RsuBandwidthPrb  = 106;

    uint16_t RsunumContiguousCc = 1;
    uint16_t RsuNumerology = 1;
    double RsuTxPower = 23.0; // 단위dBm
    // double Rsux = pow(10, RsuTxPower / 10); // to mW

    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);

    // RSU band 설정
    CcBwpCreator RsuCcBwpCreator;
    CcBwpCreator::SimpleOperationBandConf RsuBandConf(RsuFrequencyBand,
                                                      RsuBandwidthHz,
                                                      RsunumContiguousCc,
                                                      BandwidthPartInfo::V2V_Highway);
    OperationBandInfo RsuBand = RsuCcBwpCreator.CreateOperationBandContiguousCc(RsuBandConf);

    nrHelper->InitializeOperationBand(&RsuBand);
    BandwidthPartInfoPtrVector RsuBwp = CcBwpCreator::GetAllBwps({RsuBand});

    // RSU 안테나 설정
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(RsuTxPower)); // dBm그대로 넣는듯

    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId()); // 이거 필수임 이유는 찾아봐 todo
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    uint8_t bwpIdForGbrMcptt = 0;
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    // following parameter has no impact at the moment because:
    // 1. No support for PQI based mapping between the application and the LCs
    // 2. No scheduler to consider PQI
    // However, till such time all the NR SL examples should use GBR_MC_PUSH_TO_TALK
    // because we hard coded the PQI 65 in UE RRC.
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    std::vector<ObjectFactory> macSlFactory;
    ObjectFactory slfactory;
    slfactory.SetTypeId(NrSlUeMac::GetTypeId());
    macSlFactory.push_back(slfactory);

    NetDeviceContainer rsuNetDev =
        nrHelper->InstallUeDevice(rsuNodeContainer, RsuBwp, macSlFactory);

    nrHelper->GetUePhy(rsuNetDev.Get(0), 0)->SetNoiseFigure(7.0);
    // 설정 적용
    for (auto it = rsuNetDev.Begin(); it != rsuNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
        // Update the RRC config.Must be called only once.
    }

    // ue안테나 설정
    NetDeviceContainer ueSlNetDev =
        nrHelper->InstallUeDevice(ueNodeContainer, RsuBwp, macSlFactory);

    nrHelper->GetUePhy(ueSlNetDev.Get(0), 0)->SetNoiseFigure(7.0);
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->GetUePhy(ueSlNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(ueTxPower));

    DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0))->UpdateConfig(); // todo: obu sl

    NetDeviceContainer SlNetDev;
    SlNetDev.Add(ueSlNetDev);
    SlNetDev.Add(rsuNetDev);

    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

    nrSlHelper->PrepareUeForSidelink(SlNetDev, bwpIdContainer);

    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitmap =
        {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1}; // The sidelink time resource bitmap

    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100);    //!< Start of the sensing window in milliseconds.
    ptrFactory->SetSlSelectionWindow(5);    //!< End of the selection window in number of slots.
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(50);
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);

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
    // 풀을 여러개 쓸 수 있지만 우리는 영상 데이터를 전송하는 거니까 풀 하나만 쓰는게 맞을 듯

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = RsuNumerology;
    bwp.symbolsPerSlots = 14; // ofdm symbol
    bwp.rbPerRbg = 1;         // Resource block per resource block group
    bwp.bandwidth = RsuBandwidthHz;

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

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    // Array for BWPs. Here we will iterate over the BWPs, which
    // we want to use for SL.
    for (const auto &it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|DL|UL|UL|UL|UL|UL|UL|";

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    slUeSelectedPreConfig.slProbResourceKeep = 0;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    nrSlHelper->InstallNrSlPreConfiguration(SlNetDev, slPreConfigNr);

    // 진짜 시작todo:
    // ===============================================================================

    NodeContainer nodes(server);
    NodeContainer routers(pgw, rsu, router);

    // 여기서 p2p를 쓸지 csma를 쓸지 결정해야할듯
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("5ms"));
    NetDeviceContainer pgwToRouterNetDev = p2ph.Install(pgw, router);
    NetDeviceContainer rsuToRouterNetDev = p2ph.Install(rsu, router);
    NetDeviceContainer routerToServerNetDev = p2ph.Install(router,server);



    // 인터넷 설정
    InternetStackHelper internet;
    internet.Install(ueNodeContainer);
    internet.Install(nodes);
    internet.Install(routers);

    // ip설정===================================================
    Ipv4AddressHelper Ipv4h;

    // rsu router
    Ipv4h.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer iic1 = Ipv4h.Assign(rsuToRouterNetDev); //10.1.1.1
    Ipv4Address rsuRouterIp=iic1.GetAddress(0);
    Ipv4Address routerRsuIp=iic1.GetAddress(1);
    std::cout<<"rsuRouterIp : "<<rsuRouterIp<<std::endl;
    std::cout<<"routerRsuIp : "<<routerRsuIp<<std::endl;

    // pgw router
    Ipv4h.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer iic2 = Ipv4h.Assign(pgwToRouterNetDev); //10.1.1.2
    Ipv4Address pgwRouterIp= iic2.GetAddress(0);
    Ipv4Address routerPgwIp= iic2.GetAddress(1);
    std::cout<<"pgwRouterIp : "<<pgwRouterIp<<std::endl;
    std::cout<<"routerPgwIp : "<<routerPgwIp<<std::endl;


    // router server
    Ipv4h.SetBase("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer iic3 = Ipv4h.Assign(routerToServerNetDev); //10.1.1.3
    Ipv4Address routerServerIp = iic3.GetAddress(0);
    Ipv4Address serverRouterIp = iic3.GetAddress(1);
    std::cout<<"serverRouterIp : "<<serverRouterIp<<std::endl;
    std::cout<<"routerServerIp : "<<routerServerIp<<std::endl;

    // ue uu
    Ipv4InterfaceContainer ueUuIface = epcHelper->AssignUeIpv4Address(ueUuNetDev);
    std::cout<<"ueUUIp : "<<ueUuIface.GetAddress(0)<<std::endl;

    // ue sl
    // Ipv4InterfaceContainer ueSlIface = epcHelper->AssignUeIpv4Address(SlNetDev);
    Ipv4h.SetBase("192.168.10.0", "255.255.255.0");
    Ipv4InterfaceContainer ueSlIface = Ipv4h.Assign(SlNetDev);
    Ipv4Address ueSlIp = ueSlIface.GetAddress(0);
    Ipv4Address rsuSlIp = ueSlIface.GetAddress(1);
    std::cout<<"ueSlIp : "<<ueSlIp<<std::endl;
    std::cout<<"rsuSlIp : "<<rsuSlIp<<std::endl;


    // 라우팅======================================================
    // Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    nrHelper->AttachToClosestEnb(ueUuNetDev, gnbNetDev); // 이거는 eps베어러 생성 기지국과 연결해줌

    //인터페이스 확인하는 코드=============================================================================
    // Ptr<Ipv4> pgwIpv4 = rsu->GetObject<Ipv4>();
    // Ptr<NetDevice> deviceOnPgw = rsuToRouterNetDev.Get(0);
    // uint32_t temp = pgwIpv4->GetInterfaceForDevice(deviceOnPgw);
    // std::cout << "PGW to Router Interface Index: " << temp << std::endl;

    Ipv4StaticRoutingHelper Ipv4RoutingHelper;
    uint32_t ueUuItf = 1;
    uint32_t ueSlItf = 2;
    uint32_t rsuSlItf = 2;
    uint32_t rsuRouterItf = 1;
    uint32_t routerServerItf = 3;
    uint32_t serverRouterItf = 1;
    uint32_t routerPgwItf = 2;
    uint32_t routerRsuItf = 1;
    uint32_t pgwRouterItf = 3;



    // ue 라우팅
    Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), ueUuItf);
    ueStaticRouting->SetDefaultMulticastRoute(ueSlItf);

    // Ptr<NetDevice> ueSlNetDevice = ue->GetDevice(ueSlItf);
    // Mac48Address ueSlMacAddress = Mac48Address::ConvertFrom(ueSlNetDevice->GetAddress());

    // rsu 라우팅
    Ptr<Ipv4StaticRouting> rsuStaticRouting = Ipv4RoutingHelper.GetStaticRouting(rsu->GetObject<Ipv4>());
    // rsuStaticRouting->SetDefaultRoute(routerRsuIp, rsuRouterItf);
    rsuStaticRouting->AddMulticastRoute(Ipv4Address("192.168.10.1"), // all sources (ASM)
                                   groupAddress4,
                                   /* input interface */ rsuSlItf,
                                   /* output interfaces */ std::vector<uint32_t>{ rsuRouterItf });

    rsuStaticRouting->AddNetworkRouteTo(Ipv4Address("192.168.10.0"),
                                         Ipv4Mask("255.255.255.0"),
                                         rsuSlItf);

    rsuStaticRouting->AddHostRouteTo(Ipv4Address("192.168.10.1"), rsuSlItf);




    // // 2. RSU 노드의 Ipv4 스택에서 Sidelink에 해당하는 'Ipv4Interface'를 가져옵니다.
    // Ptr<Node> rsuNode = rsu;
    // Ptr<Ipv4L3Protocol> rsuIpv41 = rsuNode->GetObject<Ipv4L3Protocol>();
    // Ptr<Ipv4Interface> rsuSlInterface = rsuIpv41->GetInterface(rsuSlItf); // rsuSlItf는 RSU의 Sidelink 인터페이스 인덱스
    //
    //
    // // 3. 해당 Ipv4Interface에서 ArpCache를 가져옵니다.
    // Ptr<ArpCache> rsuArpCache = rsuSlInterface->GetArpCache();
    // Mac48Address hardcodedMac("04:06:C0:A8:0A:01");
    //
    // // 4. ArpCache에 정적 항목을 'Add -> Set -> Mark' 3단계로 추가합니다.
    // if (rsuArpCache) // rsuArpCache가 유효한지 확인
    // {
    //     // 1단계: IP 주소로 빈 엔트리 생성
    //     ArpCache::Entry* entry = rsuArpCache->Add(Ipv4Address("192.168.10.1"));
    //
    //     // 2단계: 생성된 엔트리에 MAC 주소 설정
    //     entry->SetMacAddress(hardcodedMac);
    //
    //     // 3단계: 엔트리를 영구(PERMANENT) 상태로 변경
    //     entry->MarkPermanent();
    // }

    // pgw 라우팅
    Ptr<Ipv4StaticRouting> pgwStaticRouting = Ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
    pgwStaticRouting->SetDefaultRoute(routerPgwIp, pgwRouterItf);

    // server 라우팅
    Ptr<Ipv4StaticRouting> serverStaticRouting = Ipv4RoutingHelper.GetStaticRouting(server->GetObject<Ipv4>());
    serverStaticRouting->SetDefaultRoute(routerServerIp, serverRouterItf);
    // serverStaticRouting->AddNetworkRouteTo(
    //     Ipv4Address("7.0.0.0"),
    //     Ipv4Mask("255.0.0.0"),
    //     routerServerIp,
    //     serverRouterItf
    // );

    // router 라우팅
    Ptr<Ipv4StaticRouting> routerStaticRouting = Ipv4RoutingHelper.GetStaticRouting(router->GetObject<Ipv4>());

    routerStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                           Ipv4Mask("255.0.0.0"), // EPC망의 서브넷 마스크
                                           pgwRouterIp,
                                           /* 라우터의 PGW 방향 인터페이스 */routerPgwItf);

    // -> UE의 Sidelink망으로 가는 길은 RSU를 통한다.
    routerStaticRouting->AddNetworkRouteTo(Ipv4Address("192.168.10.0"),
                                           Ipv4Mask("255.255.255.0"),
                                           rsuRouterIp,
                                           /* 라우터의 RSU 방향 인터페이스 */routerRsuItf);

    // -> 멀티캐스트 트래픽은 서버 쪽으로 포워딩한다.
    //    (정적 멀티캐스트 라우팅 설정)
    routerStaticRouting->AddMulticastRoute(Ipv4Address("0.0.0.0"), // 모든 소스
                                         groupAddress4,         // 멀티캐스트 그룹
                                         /* 라우터의 RSU 방향 인터페이스 (입력) */routerRsuItf,
                                         /* 라우터의 서버 방향 인터페이스 목록 (출력) */std::vector<uint32_t> {routerServerItf});

    //포워딩
    Ptr<Ipv4> rsuIpv4 = rsu->GetObject<Ipv4>();
    rsuIpv4->SetAttribute("IpForward", BooleanValue(true));
    for (uint32_t i =0; i < rsuIpv4->GetNInterfaces(); i++)
    {
        rsuIpv4->SetForwarding(i,true);
    }


    Ptr<Ipv4> routerIpv4 = router->GetObject<Ipv4>();
    routerIpv4->SetAttribute("IpForward", BooleanValue(true));
    for (uint32_t i =0; i < routerIpv4->GetNInterfaces(); i++)
    {
        routerIpv4->SetForwarding(i,true);
    }

    StackHelper stackHelper;
    stackHelper.PrintRoutingTable(router);
    stackHelper.PrintRoutingTable(rsu);
    stackHelper.PrintRoutingTable(pgw);

    // sidelink 무선 베어러 설정=====================================================================
    Ptr<LteSlTft> tft;
    uint32_t dstL2Id = 255;
    Time delayBudget = Seconds(0);

    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(100);
    slInfo.m_pdb = delayBudget;
    slInfo.m_dynamic = false;
    slInfo.m_harqEnabled = true;

    tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, slInfo);
    nrSlHelper->ActivateNrSlBearer(Seconds(0.0), ueSlNetDev, tft);

    // sidelink 무선 베어러 설정=====================================================================
    Ptr<LteSlTft> tft1;

    SidelinkInfo slInfo1;
    slInfo1.m_castType = SidelinkInfo::CastType::Unicast;
    slInfo1.m_dstL2Id = dstL2Id;
    slInfo1.m_rri = MilliSeconds(100);
    slInfo1.m_pdb = delayBudget;
    slInfo1.m_dynamic = false;
    slInfo1.m_harqEnabled = true;

    tft1 = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, ueSlIp, slInfo1);
    nrSlHelper->ActivateNrSlBearer(Seconds(0.0), rsuNetDev, tft1);




    // sidelink 무선 베어러 설정 끝=====================================================================

    // // Uu PHY에서 RSRP 측정 콜백 연결 (gNb와의 Uu 통신)
    Ptr<NrUeNetDevice> ueUuDev = DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0));
    // Get the first PHY (BWP) from the Uu NetDevice
    Ptr<NrUePhy> ueUuPhy = ueUuDev->GetPhy(0);
    ueUuPhy->TraceConnectWithoutContext("ReportRsrp", MakeCallback(&UeMeasCallback));

    // Uu PHY에서 RSRP 측정 콜백 연결 (gNb와의 Uu 통신)
    Ptr<NrUeNetDevice> rsutemp = DynamicCast<NrUeNetDevice>(ueSlNetDev.Get(0));
    // Get the first PHY (BWP) from the Uu NetDevice
    Ptr<NrUePhy> rsutemp2 = rsutemp->GetPhy(0);
    rsutemp2->TraceConnectWithoutContext("ReportSlRsrp", MakeCallback(&UeSlMeasCallback));

    for (uint32_t i = 0; i < SlNetDev.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(SlNetDev.Get(i));
        Ptr<NrUePhy> phy = ueDev->GetPhy(0);
        phy->GetNrSlUeCphySapProvider()->EnableUeSlRsrpMeasurements();
    }

    //  todo: 앱설치 =============================================================
    ApplicationContainer ueAppContainer;
    ApplicationContainer serverAppContainer;
    ApplicationContainer rsuAppContainer;

    u_int16_t serverPort = 5000;
    // u_int16_t rsuSlPort = 6000;

    // u_int16_t clientPort = 4000;
    // AdaptiveUdpClientk adaptiveUdpClientk(rsuIpv4,serverIpv4, serverPort);
    // adaptiveUdpClientk.SetAttribute("Interval",TimeValue(Seconds(3)));

    Ptr<UdpKohServer> serverApp = CreateObject<UdpKohServer>();
    server->AddApplication(serverApp);
    serverApp->SetAttribute("Port", UintegerValue(serverPort));
    serverApp->SetStartTime(Seconds(1.0));
    serverApp->SetStopTime(simTime);

    Ptr<UdpKohClient> clientApp = CreateObject<UdpKohClient>();
    clientApp->SetAttribute("MaxPackets", UintegerValue(100));
    clientApp->SetAttribute("Interval", TimeValue(Seconds(1.0)));
    clientApp->SetAttribute("PacketSize", UintegerValue(100));
    clientApp->SetAttribute("slServerAddress", AddressValue(groupAddress4));
    clientApp->SetAttribute("slServerPort", UintegerValue(serverPort));
    clientApp->SetAttribute("uuServerAddress", AddressValue(serverRouterIp));
    clientApp->SetAttribute("uuServerPort", UintegerValue(serverPort));

    ue->AddApplication(clientApp);
    clientApp->SetStartTime(Seconds(2.0));
    clientApp->SetStopTime(simTime);

    // rsu 애플리케이션 sidelink rsrp
    Ptr<OnOffApplication> rsuApp = CreateObject<OnOffApplication>();
    rsuApp->SetAttribute("PacketSize", UintegerValue(1));
    rsuApp->SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.01]"));
    rsuApp->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.09]"));
    rsuApp->SetAttribute("Protocol", TypeIdValue(UdpSocketFactory::GetTypeId()));
    Address remoteAddress(InetSocketAddress(ueSlIp, 5555));
    rsuApp->SetAttribute("Remote", AddressValue(remoteAddress));

    rsu->AddApplication(rsuApp);
    rsuApp->SetStartTime(Seconds(0.0));
    rsuApp->SetStopTime(simTime);




    // todo:여기다가포트랑 주소 넣어야함
    // clientApp->setAddressSlUu(gnbServerIpv4, serverPort, groupAddress6, rsuSlPort);


    // 인터페이스 바꾸는거 그냥 예시
    clientApp->setInterface(ueUuNetDev.Get(0), ueSlNetDev.Get(0));
    Simulator::Schedule(Seconds(10.0), &UdpKohClient::changeInterface, clientApp);
    Simulator::Schedule(Seconds(60.0), &UdpKohClient::changeInterface, clientApp);

    Ptr<Ipv4> ipv4 = clientApp->GetNode()->GetObject<Ipv4>();
    for (uint32_t ifIndex = 0; ifIndex < ipv4->GetNInterfaces(); ++ifIndex)
    {
        for (uint32_t addrIndex = 0; addrIndex < ipv4->GetNAddresses(ifIndex); ++addrIndex)
        {
            auto ifAddr = ipv4->GetAddress(ifIndex, addrIndex);
            std::cout << "Iface " << ifIndex << ", Addr " << addrIndex << ": "
                      << ifAddr.GetAddress() << "/"  << std::endl;
        }
    }
    // main 함수 내부, 인터넷 스택 설치 이후

    // RSU 노드의 Ipv4 L3 프로토콜의 "Rx" Trace Source에 콜백 함수를 연결합니다.
    // "Rx"는 IP 계층에서 패킷을 수신하는 이벤트입니다.
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(rsu->GetId()) +
                                      "/$ns3::Ipv4L3Protocol/Rx",
                                  MakeCallback(&Ipv4PacketTraceAtRsu));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(pgw->GetId()) +
                                      "/$ns3::Ipv4L3Protocol/Rx",
                                  MakeCallback(&Ipv4PacketTraceAtPgw));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(router->GetId()) +
                                  "/$ns3::Ipv4L3Protocol/Rx",
                              MakeCallback(&Ipv4PacketTraceAtRouter));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(server->GetId()) +
                              "/$ns3::Ipv4L3Protocol/Rx",
                          MakeCallback(&Ipv4PacketTraceAtServer));
    // Config::ConnectWithoutContext("/NodeList/" + std::to_string(ue->GetId()) +
    //                           "/$ns3::Ipv4L3Protocol/Rx",
    //                       MakeCallback(&Ipv4PacketTraceAtUe));

    Simulator::Schedule(Seconds(0.0), &PrintUeInfo, ueNodeContainer.Get(0));

    // LogComponentEnable("UdpSocketImpl", LOG_LEVEL_INFO);
    // LogComponentEnable("UdpSocketImpl", LOG_LEVEL_FUNCTION);

    // --- 시뮬레이션 실행 ---
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}
