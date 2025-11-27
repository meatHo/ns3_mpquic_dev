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
#include <ns3/pointer.h>
using namespace ns3;

struct WaypointData
{
    double time;
    double x;
    double y;
    double z;
    double speed;
};


void
UeMeasCallback(uint16_t cellId, uint16_t IMSI, uint16_t RNTI, double RSRP, uint8_t BWPId)
{
    std::cout << "📶 [Meas] cellId=" << cellId << " IMSI=" << IMSI << " BWPId=" << BWPId
              << "  RNTI=" << RNTI << " RSRP=" << RSRP << " dB\n";
}

void
UeSlMeasCallback(uint16_t RNTI, uint32_t L2ID, double RSRP)
{
    std::cout << "📶 [Meas] RNTI=" << RNTI << " L2ID=" << L2ID << " RSRP=" << RSRP << " dB\n";
}

#include <ns3/spectrum-model.h>
#include <ns3/spectrum-value.h>

#include <cmath>

#include "ns3/spectrum-phy.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/net-device.h"
#include "ns3/node.h"



// UE의 위치와 속도를 출력하는 함수
void
PrintUeInfo(Ptr<Node> ueNode)
{
    Ptr<MobilityModel> mob = ueNode->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    Vector vel = mob->GetVelocity();

    NS_LOG_UNCOND("Time: " << Simulator::Now().GetSeconds() << "s");
    NS_LOG_UNCOND("UE Position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z);
    NS_LOG_UNCOND("UE Velocity: x=" << vel.x << ", y=" << vel.y << ", z=" << vel.z << " (m/s)");
    Simulator::Schedule(Seconds(1.0), &PrintUeInfo, ueNode);
}

void
Ipv6PacketTraceAtPgw(Ptr<const Packet> packet, Ptr<Ipv6> ipv6, uint32_t interfaceIndex)
{
    Ipv6Header ipv6Header;
    packet->PeekHeader(ipv6Header);

    std::cout << "[PGW Packet Trace] Time: " << Simulator::Now().GetSeconds() << "s"
              << " | Interface: " << interfaceIndex << " | Size: " << packet->GetSize() << " bytes"
              << std::endl;
}



namespace ns3
{

class Socket;
class Packet;

class TcpClient : public Application
{
public:
    static TypeId GetTypeId();

    TcpClient();
    ~TcpClient() override;

    uint64_t GetTotalTx() const;

    void changeInterface();
    void setInterface(Ptr<NetDevice> uu, Ptr<NetDevice> sl);
    // [수정] RSU의 Sidelink IP 주소(Next Hop)를 받기 위한 파라미터 추가

    void ConnectionSucceeded(Ptr<Socket> socket);
    void ConnectionFailed(Ptr<Socket> socket);

private:
    void StartApplication() override;
    void StopApplication() override;
    void Send();
    void SelectInterface(Ptr<Socket> socket);
    void HandleRecv(Ptr<Socket> socket);

    TracedCallback<Ptr<const Packet>> m_txTrace;
    TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_txTraceWithAddresses;

    uint32_t m_count;
    Time m_interval;
    uint32_t m_size;

    uint32_t m_sent;
    uint64_t m_totalTx;
    Ptr<Socket> m_uuSocket;
    Ptr<Socket> m_slSocket;
    Ptr<Socket> m_sendSocket;
    Ptr<Socket> m_recvSocket;
    Address m_slServerAddress;
    uint16_t m_slServerPort;
    uint16_t m_recvPort;
    Address m_uuServerAddress;
    uint16_t m_uuServerPort;
    Ptr<NetDevice> m_devUu, m_devSl;


    uint8_t m_tos;
    EventId m_sendEvent;

#ifdef NS3_LOG_ENABLE
    std::string m_peerAddressString;
#endif
};

} // namespace ns3


TypeId
TcpClient::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpClient")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<TcpClient>()
            .AddAttribute(
                "MaxPackets",
                "The maximum number of packets the application will send (zero means infinite)",
                UintegerValue(100),
                MakeUintegerAccessor(&TcpClient::m_count),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute("Interval",
                          "The time to wait between packets",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&TcpClient::m_interval),
                          MakeTimeChecker())
            .AddAttribute("uuServerAddress",
                          "The destination Address of the outbound packets",
                          AddressValue(),
                          MakeAddressAccessor(&TcpClient::m_uuServerAddress),
                          MakeAddressChecker())
            .AddAttribute("uuServerPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&TcpClient::m_uuServerPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("recvPort",
                          "The destination port of the outbound packets",
                          UintegerValue(100),
                          MakeUintegerAccessor(&TcpClient::m_recvPort),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Tos",
                          "The Type of Service used to send IPv4 packets. "
                          "All 8 bits of the TOS byte are set (including ECN bits).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&TcpClient::m_tos),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("PacketSize",
                          "Size of packets generated. The minimum packet size is 12 bytes which is "
                          "the size of the header carrying the sequence number and the time stamp.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&TcpClient::m_size),
                          MakeUintegerChecker<uint32_t>(12, 65507))
            .AddTraceSource("Tx",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&TcpClient::m_txTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("TxWithAddresses",
                            "A new packet is created and sent",
                            MakeTraceSourceAccessor(&TcpClient::m_txTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

TcpClient::TcpClient()
{
    m_sent = 0;
    m_totalTx = 0;
    m_uuSocket = nullptr;
    m_slSocket = nullptr;
    m_sendEvent = EventId();
}

TcpClient::~TcpClient()
{
}

void
TcpClient::StartApplication()
{

    TypeId tid = TypeId::LookupByName("ns3::TcpSocketFactory");
    // listening 소켓 설정
    m_recvSocket = Socket::CreateSocket(GetNode(), tid);
    Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_recvPort);
    if (m_recvSocket->Bind(local) == -1)
    {
        NS_FATAL_ERROR("UdpRelay: Failed to bind In-Socket");
    }// todo: handle read 안만들음 나중에 콜백으로 이으셈 필요하면

    // Uu 소켓 설정
    m_uuSocket = Socket::CreateSocket(GetNode(), tid);
    m_uuSocket->BindToNetDevice(m_devUu); // Connect 전에 Bind


    //tcp
    m_uuSocket->SetConnectCallback(MakeCallback(&TcpClient::ConnectionSucceeded, this),
                               MakeCallback(&TcpClient::ConnectionFailed, this));


    if (Ipv6Address::IsMatchingType(m_uuServerAddress))
    {
        NS_LOG_UNCOND("uu socket connection started");
        m_uuSocket->Connect(
            Inet6SocketAddress(Ipv6Address::ConvertFrom(m_uuServerAddress), m_uuServerPort));
    }


    // 수신 콜백 설정 (양방향 통신 시 필요)
    m_uuSocket->SetRecvCallback(MakeCallback(&TcpClient::HandleRecv, this));


    // 초기 인터페이스는 Uu로 설정
    m_sendSocket = m_uuSocket;
    NS_LOG_UNCOND("Client starts with uu interface.");

    // m_sendEvent = Simulator::Schedule(Seconds(0.0), &UdpClient::Send, this);
}

void
TcpClient::ConnectionSucceeded(Ptr<Socket> socket)//tcp
{
    NS_LOG_UNCOND("Connection Succeeded.");

    // 최초 연결이 성공하면 Send() 함수 스케줄링 시작
    if (!m_sendEvent.IsRunning())
    {
        NS_LOG_UNCOND("First connection established. Starting to send data.");
        m_sendEvent = Simulator::Schedule(Seconds(0.0), &TcpClient::Send, this);
    }
}

void
TcpClient::ConnectionFailed(Ptr<Socket> socket)
{
    NS_LOG_UNCOND("Connection Failed.");
    if (socket == m_uuSocket)
    {
        NS_LOG_UNCOND("UU Socket connection failed.");
    }
}


void
TcpClient::HandleRecv(Ptr<Socket> socket)
{

    Ptr<Packet> packet;
    // ★★★ RecvFrom 대신 Recv 사용
    while ((packet = socket->Recv()))
    {
        if (packet->GetSize() == 0)
        {
            break;
        }
        uint8_t* buffer = new uint8_t[packet->GetSize() + 1];
        packet->CopyData(buffer, packet->GetSize());
        buffer[packet->GetSize()] = '\0';
        NS_LOG_UNCOND("Received a " << packet->GetSize() << " bytes packet: " << buffer);
        delete[] buffer;
    }
}


void
TcpClient::StopApplication()
{
    Simulator::Cancel(m_sendEvent);
}


void
TcpClient::Send()
{
    NS_LOG_UNCOND("TCPCLIENT::SEND");
    NS_ASSERT(m_sendEvent.IsExpired());

    SeqTsHeader seqTs;
    seqTs.SetSeq(m_sent);
    Ptr<Packet> p = Create<Packet>(m_size - seqTs.GetSerializedSize());
    p->AddHeader(seqTs);

    // ★★★ SendTo 대신 Send 사용 (연결된 소켓이므로 목적지 지정 불필요)
    if ((m_sendSocket->Send(p)) >= 0)
    {
        ++m_sent;
        m_totalTx += p->GetSize();
        NS_LOG_UNCOND("Sent a " << p->GetSize() << " bytes packet. Total sent: " << m_sent);
    }
    else
    {
        NS_LOG_UNCOND("Error while sending packet.");
    }

    if (m_sent < m_count || m_count == 0)
    {
        m_sendEvent = Simulator::Schedule(m_interval, &TcpClient::Send, this);
    }
}

uint64_t
TcpClient::GetTotalTx() const
{
    return m_totalTx;
}

void
TcpClient::SelectInterface(Ptr<Socket> socket)
{
    // 여기에 rsu가 좋은지 gnb가 좋은지 고르는 함수 추가
    m_sendSocket = socket;
}

// [핵심 수정] changeInterface() 함수
void
TcpClient::changeInterface()
{
    if (m_sendSocket == m_uuSocket)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s: ---> Switching client interface to SL socket <---");

        m_sendSocket = m_slSocket;
    }
    else
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s: ---> Switching client interface to UU socket <---");

        m_sendSocket = m_uuSocket;
    }
}

void
TcpClient::setInterface(Ptr<NetDevice> uu, Ptr<NetDevice> sl)
{
    m_devSl = sl;
    m_devUu = uu;
}


// udp 서버 설정===============================================================
struct clientInfo
{
    Address address;
    uint32_t lastSequenceNum;
    Time connectionTime;
    uint32_t RTT;
    float_t packetLossRate;
    uint64_t totalBytesReceived;
    //TCP
    Ptr<Socket> socket;
};

class UdpServerk : public Application
{
  public:
    static TypeId GetTypeId();
    UdpServerk();
    ~UdpServerk() override {};
    void SetPacketWindowSize(uint16_t size);
    uint16_t GetPacketWindowSize() const;

  private:
    std::map<uint16_t, clientInfo> clients;
    void StartApplication() override;
    void StopApplication() override;

    uint16_t m_port; //!< Port on which we listen for incoming packets.
    uint8_t m_tos;   //!< The packets Type of Service
    // Ptr<Socket> m_socket;            //!< IPv4 Socket
    Ptr<Socket> m_socket6;           //!< IPv6 Socket

    uint64_t m_received;             //!< Number of received packets
    PacketLossCounter m_lossCounter; //!< Lost packet counter
    uint16_t m_nextClientId;

    /// Callbacks for tracing the packet Rx events
    TracedCallback<Ptr<const Packet>> m_rxTrace;

    /// Callbacks for tracing the packet Rx events, includes source and destination addresses
    TracedCallback<Ptr<const Packet>, const Address&, const Address&> m_rxTraceWithAddresses;

    void HandleRead(Ptr<Socket> socket);

    // --- TCP 동작을 위해 추가/수정된 함수들 ---
    void HandleAccept(Ptr<Socket> s, const Address& from);

    Ptr<Socket> m_listenSocket6;
    std::map<Ptr<Socket>, uint16_t> m_socketToClientId;
};

NS_OBJECT_ENSURE_REGISTERED(UdpServerk); // GetTypeId 위에 추가

TypeId
UdpServerk::GetTypeId()
{
    static TypeId tid =
        TypeId("UdpServerk")
            .SetParent<Application>()
            .SetGroupName("Applications")
            .AddConstructor<UdpServerk>()
            // 포트는 Attribute로 설정하는 것이 표준입니다.
            .AddAttribute("Port",
                          "Port on which we listen for incoming packets.",
                          UintegerValue(9000), // 기본 포트값
                          MakeUintegerAccessor(&UdpServerk::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Tos",
                          "The Type of Service used to send IPv4 packets. "
                          "All 8 bits of the TOS byte are set (including ECN bits).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&UdpServerk::m_tos),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("PacketWindowSize",
                          "The size of the window used to compute the packet loss. This value "
                          "should be a multiple of 8.",
                          UintegerValue(32),
                          MakeUintegerAccessor(&UdpServerk::GetPacketWindowSize,
                                               &UdpServerk::SetPacketWindowSize),
                          MakeUintegerChecker<uint16_t>(8, 256))
            .AddTraceSource("Rx",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&UdpServerk::m_rxTrace),
                            "ns3::Packet::TracedCallback")
            .AddTraceSource("RxWithAddresses",
                            "A packet has been received",
                            MakeTraceSourceAccessor(&UdpServerk::m_rxTraceWithAddresses),
                            "ns3::Packet::TwoAddressTracedCallback");
    return tid;
}

UdpServerk::UdpServerk()
    : m_received(0),
      m_lossCounter(0)
{
    m_nextClientId = 0;
}

void
UdpServerk::StartApplication()
{
    // 소켓 만들어서 대입
    std::cout << "udp serverk StartApplication" << std::endl;
    m_listenSocket6 = Socket::CreateSocket(GetNode(), TypeId(TcpSocketFactory::GetTypeId()));
    Inet6SocketAddress local = Inet6SocketAddress(Ipv6Address::GetAny(), m_port);

    if (m_listenSocket6->Bind(local) == -1)
    {
        NS_FATAL_ERROR("Failed to bind socket");
    }
    m_listenSocket6->Listen();
    m_listenSocket6->ShutdownSend();
    m_listenSocket6->SetAcceptCallback(
        MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
        MakeCallback(&UdpServerk::HandleAccept, this));

}

// UdpServerk 클래스 내부
void UdpServerk::HandleAccept(Ptr<Socket> newSocket, const Address& from)
{
    NS_LOG_UNCOND("✅ Server: Accepted new connection from " << Inet6SocketAddress::ConvertFrom(from).GetIpv6());
    uint16_t newId = m_nextClientId++;
    clientInfo newClient;
    newClient.socket = newSocket;
    newClient.address = from;
    newClient.connectionTime = Simulator::Now();
    newClient.totalBytesReceived = 0;

    clients[newId] = newClient;
    m_socketToClientId[newSocket] = newId;

    // 새로 생성된 소켓에 데이터 수신 콜백을 설정합니다.
    newSocket->SetRecvCallback(MakeCallback(&UdpServerk::HandleRead, this));
}

void
UdpServerk::StopApplication()
{
    if (m_socket6)
    {
        m_socket6->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

uint16_t
UdpServerk::GetPacketWindowSize() const
{
    return m_lossCounter.GetBitMapSize();
}

void
UdpServerk::SetPacketWindowSize(uint16_t size)
{
    std::cout << "bitmap size:" << size << std::endl;
    m_lossCounter.SetBitMapSize(size);
}

void UdpServerk::HandleRead(Ptr<Socket> socket)
{
    // 소켓을 이용해 클라이언트 ID를 찾습니다.
    uint16_t clientId = m_socketToClientId[socket];

    Ptr<Packet> packet;
    while ((packet = socket->Recv()))
    {
        if (packet->GetSize() == 0) // 연결 종료 감지
        {
            NS_LOG_UNCOND("👋 Server: Client " << clientId << " disconnected.");

            // 두 개의 map에서 모두 해당 클라이언트 정보를 제거합니다.
            clients.erase(clientId);
            m_socketToClientId.erase(socket);
            break;
        }

        // 해당 클라이언트의 수신 데이터양 업데이트
        clients[clientId].totalBytesReceived += packet->GetSize();
        NS_LOG_UNCOND("📦 Server: Received " << packet->GetSize() << " bytes from Client " << clientId
                     << ". Total received: " << clients[clientId].totalBytesReceived << " bytes.");

        // 에코 응답
        socket->Send(packet->Copy());
    }
}


int
main(void)
{
    // 1. CSV 파일 읽기
    // ==========================================================
    std::vector<WaypointData> waypoints;
    std::string csvFileName = "/home/kiho/ns-3-dev/scratch/final_3d_trace.csv";
    std::ifstream file(csvFileName);

    if (!file.is_open())
    {
        std::cout << "Could not open CSV file: " << csvFileName << std::endl;
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

    // ns3 세팅 시작
    Time simTime = Seconds(10);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    NodeContainer gnbNodeContainer;
    gnbNodeContainer.Create(1);
    NodeContainer serverNodeContainer;
    serverNodeContainer.Create(1);
    NodeContainer ueNodeContainer;
    ueNodeContainer.Create(1);

    Ptr<Node> pgw = epcHelper->GetPgwNode(); // ipv4, ipv6 둘다 설치되어 있음. 듀얼스택
    Ptr<Node> server = serverNodeContainer.Get(0);
    Ptr<Node> gnb = gnbNodeContainer.Get(0);
    Ptr<Node> ue = ueNodeContainer.Get(0);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodeContainer);
    gnbNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(900, 4288, 80.0));
    mobility.Install(serverNodeContainer);
    serverNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(1900.0, 3800.0, 60.0));

    mobility.Install(ueNodeContainer);
    ueNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(900.0, 4315.03, 59));

    // mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    // mobility.Install(ueNodeContainer);
    // ueNodeContainer.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(294.0, 4315.03, 59));
    // Ptr<WaypointMobilityModel> ueMobility =
    //     ueNodeContainer.Get(0)->GetObject<WaypointMobilityModel>();
    //
    // // 읽어온 CSV 데이터를 Waypoint로 추가
    // for (const auto& data : waypoints)
    // {
    //     Waypoint waypoint(Seconds(data.time), Vector(data.x, data.y, data.z));
    //     ueMobility->AddWaypoint(waypoint);
    // }

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
                                                      BandwidthPartInfo::UMi_StreetCanyon_nLoS); //
    gNbBandConf.m_numBwp = 1; // 1 BWP per CC
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

    // 안테나 설정
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_PREMIUM",
                                                 UintegerValue(0)); // bwp하나만 한거

    std::string pattern = "DL|DL|DL|DL|UL|UL|UL|UL|UL|UL|";
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(gNbNumerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(gNbTxPower));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Pattern", StringValue(pattern));

    // 설정 적용
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    // ue uu 설정
    NetDeviceContainer ueUuNetDev =
        nrHelper->InstallUeDevice(ueNodeContainer, gNbBwp, macUuFactory);

    double ueTxPower = 23.0;
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->GetUePhy(ueUuNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(ueTxPower));

    DynamicCast<NrUeNetDevice>(ueUuNetDev.Get(0))->UpdateConfig();

    // 진짜 시작todo:
    // ===============================================================================

    NodeContainer nodes(server);
    NodeContainer routers(pgw);

    // 여기서 p2p를 쓸지 csma를 쓸지 결정해야할듯
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2ph.SetChannelAttribute("Delay", StringValue("10ms"));
    NetDeviceContainer pgwToServerNetDev = p2ph.Install(pgw, server);

    // 인터넷 설정
    InternetStackHelper internet;
    internet.SetIpv4StackInstall(false);
    internet.Install(ueNodeContainer);
    internet.Install(nodes);
    internet.Install(routers);

    // ip설정
    Ipv6AddressHelper ipv6h;


    // pgw server
    ipv6h.SetBase("fd00:3::", Ipv6Prefix(64));
    Ipv6InterfaceContainer iic3 = ipv6h.Assign(pgwToServerNetDev);
    iic3.SetForwarding(0, true);
    Ipv6Address gnbServerIpv6 = iic3.GetAddress(1, 1);

    Ipv6InterfaceContainer ueUuIface = epcHelper->AssignUeIpv6Address(ueUuNetDev);
    nrHelper->AttachToClosestEnb(ueUuNetDev, gnbNetDev); // 이거는 eps베어러 생성 기지국과 연결해줌




    //  todo: 앱설치 =============================================================
    ApplicationContainer ueAppContainer;
    ApplicationContainer serverAppContainer;


    u_int16_t serverPort = 5000;

    // u_int16_t clientPort = 4000;
    // AdaptiveUdpClientk adaptiveUdpClientk(rsuIpv6,serverIpv6, serverPort);
    // adaptiveUdpClientk.SetAttribute("Interval",TimeValue(Seconds(3)));

    Ptr<UdpServerk> serverApp = CreateObject<UdpServerk>();
    server->AddApplication(serverApp);
    serverApp->SetAttribute("Port", UintegerValue(serverPort));
    serverApp->SetStartTime(Seconds(1.0));
    serverApp->SetStopTime(simTime);

    Ptr<TcpClient> clientApp = CreateObject<TcpClient>();
    ue->AddApplication(clientApp);
    clientApp->SetAttribute("MaxPackets", UintegerValue(1));
    clientApp->SetAttribute("Interval", TimeValue(Seconds(1.0)));
    clientApp->SetAttribute("PacketSize", UintegerValue(5000));
    clientApp->SetAttribute("uuServerAddress",AddressValue(gnbServerIpv6));
    clientApp->SetAttribute("uuServerPort",UintegerValue(serverPort));
    clientApp->SetAttribute("recvPort",UintegerValue(8080));
    clientApp->SetStartTime(Seconds(3.0));
    clientApp->SetStopTime(simTime);

    // ue pgw 라우팅
    Ipv6StaticRoutingHelper ipv6RoutingHelper;
    Ptr<Ipv6StaticRouting> ueUuStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(ue->GetObject<Ipv6>());
    ueUuStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);

    // fucking라우팅
    //  Ipv6StaticRoutingHelper ipv6RoutingHelperRsu;
    Ptr<Ipv6> ipv6 = clientApp->GetNode()->GetObject<Ipv6>();
    // Ptr<Ipv6StaticRouting> staticRouting = ipv6RoutingHelperRsu.GetStaticRouting(ipv6);
    // uint32_t slInterfaceIndex = ipv6->GetInterfaceForDevice(ueSlNetDev.Get(0));
    // staticRouting->AddNetworkRouteTo(Ipv6Address::ConvertFrom(rsuServerIpv6),
    //                                  Ipv6Prefix(128),
    //                                  Ipv6Address::ConvertFrom(temp),
    //                                  slInterfaceIndex);

    // 2. 서버 -> UE 경로 설정 (돌아오는 길) - 공식 예제의 핵심 로직
    // "서버야, UE 네트워크로 가는 패킷은 PGW로 보내라"
    Ptr<Ipv6StaticRouting> serverStaticRouting =
        ipv6RoutingHelper.GetStaticRouting(server->GetObject<Ipv6>());

    // 서버에게 "7777:f00d::/64 대역(UE 네트워크)으로 가려면,
    // 다음 목적지(Next Hop)는 fd00:3::1 (PGW)이다" 라고 알려줌
    Ipv6Address ueNetworkAddress("7777:f00d::");
    Ipv6Prefix ueNetworkPrefix(64);
    Ipv6Address pgwAddressOnP2PLink("fd00:3::1");

    // AddNetworkRouteTo(목적지 네트워크, 목적지 네트워크 Prefix, 다음 라우터 주소)
    serverStaticRouting->AddNetworkRouteTo(ueNetworkAddress, ueNetworkPrefix, 1);


    // Ptr<Ipv6> ipv6 = ue->GetObject<Ipv6>();
    for (uint32_t ifIndex = 0; ifIndex < ipv6->GetNInterfaces(); ++ifIndex)
    {
        for (uint32_t addrIndex = 0; addrIndex < ipv6->GetNAddresses(ifIndex); ++addrIndex)
        {
            auto ifAddr = ipv6->GetAddress(ifIndex, addrIndex);
            std::cout << "Iface " << ifIndex << ", Addr " << addrIndex << ": "
                      << ifAddr.GetAddress() << "/" << ifAddr.GetPrefix() << std::endl;
        }
    }

    // main 함수 내부, 인터넷 스택 설치 이후

    // RSU 노드의 IPv6 L3 프로토콜의 "Rx" Trace Source에 콜백 함수를 연결합니다.
    // "Rx"는 IP 계층에서 패킷을 수신하는 이벤트입니다.

    Config::ConnectWithoutContext("/NodeList/" + std::to_string(pgw->GetId()) +
                                      "/$ns3::Ipv6L3Protocol/Rx",
                                  MakeCallback(&Ipv6PacketTraceAtPgw));
    //   Config::ConnectWithoutContext (
    // "/NodeList/*/DeviceList/*/"
    // "$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/"
    // "NrUePhy/SpectrumPhy/RxDataTrace",
    // MakeCallback(&RxDataCallback)); // 이건 전송 받을 때만 해서 안됨

    // Config::ConnectWithoutContext(
    //     "/NodeList/*/DeviceList/*/$ns3::NrNetDevice/"
    //     "$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/ReportPowerSpectralDensity",
    //     MakeCallback(&psdCallback));

    Simulator::Schedule(Seconds(0.0), &PrintUeInfo, ueNodeContainer.Get(0));
    // Ptr<NetDevice> dev = ueUuNetDev.Get (0);
    // Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice> (dev);
    // Ptr<NrSpectrumPhy> spectrumPhy = ueDev->GetPhy (0)->GetSpectrumPhy ();
    // Ptr<NrInterference> interference = spectrumPhy->GetNrInterferenceCtrl();
    // interference->TraceConnectWithoutContext(
    //     "RssiPerProcessedChunk",
    //     MakeBoundCallback(&UeRssiPerProcessedChunk, spectrumPhy));

    // SlNetDev 컨테이너의 모든 NetDevice에 대해 반복합니다.
    // for (uint32_t i =0 ; i<SlNetDev.GetN();i++)
    // {
    //     // NetDevice를 NrUeNetDevice로 변환합니다.
    //     Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(dev);
    //     if (ueDev)
    //     {
    //         // 첫 번째 BWP(Bandwidth Part)의 PHY 객체를 가져옵니다.
    //         Ptr<NrSpectrumPhy> spectrumPhy = ueDev->GetPhy(0)->GetSpectrumPhy();
    //
    //         // Sidelink Interference 객체를 가져옵니다.
    //         Ptr<NrSlInterference> interference = spectrumPhy->GetSlInterference();
    //
    //         // 현재 어떤 노드의 어떤 객체에 연결하는지 로그를 출력합니다. (get()으로 주소 확인)
    //         NS_LOG_UNCOND("Node " << dev->GetNode()->GetId()
    //                               << ": Connecting trace to Interference object at address "
    //                               << &(*interference));
    //
    //         // "SlRssiPerProcessedChunk" 트레이스 소스에 콜백 함수를 연결합니다.
    //         interference->TraceConnectWithoutContext(
    //             "SlRssiPerProcessedChunk",
    //             MakeBoundCallback(&UeSlRssiPerProcessedChunk, spectrumPhy));
    //     }
    // }

    // --- 시뮬레이션 실행 ---
    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
}