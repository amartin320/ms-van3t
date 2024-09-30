
#include "roadside-its-station.h"
#include "ns3/core-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/traci-module.h"
#include "ns3/socket.h"
#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/geonet.h"
#include "ns3/BSContainer.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"
#include "ns3/emu-fd-net-device-helper.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-module.h"



namespace ns3
{
    NS_LOG_COMPONENT_DEFINE("RoadsideItsStation");
    
    RoadsideItsStation::RoadsideItsStation () : ItsStation()
    {
        NS_LOG_FUNCTION(this);
        m_emulation = false;
        m_netDevice_emu = new NetDeviceContainer();
        m_sock_emu = nullptr;
        m_udp_server_ip = "";
        m_dst_port = 0;
        m_cams_received = 0;
    }

    RoadsideItsStation::~RoadsideItsStation ()
    {
        NS_LOG_FUNCTION(this);
    }

    RoadsideItsStation::RoadsideItsStation (std::string v2x_technology) : ItsStation(v2x_technology)
    {
        NS_LOG_FUNCTION(this);
        m_emulation = false;
        m_sock_emu = nullptr;
        m_netDevice_emu = new NetDeviceContainer();
        m_udp_server_ip = "";
        m_dst_port = 0;
    }

    void
    RoadsideItsStation::SetNetDeviceEmu(NetDeviceContainer * netDeviceContainer) {
        NS_LOG_FUNCTION(this << netDeviceContainer);
       m_netDevice_emu = netDeviceContainer;
    }

    NetDeviceContainer *
    RoadsideItsStation::GetNetDeviceEmu() {
        NS_LOG_FUNCTION(this);
        return m_netDevice_emu;
    }

    void
    RoadsideItsStation::ConfigureEmulation (std::string udpServerIP, uint16_t dstPort) 
    {
        NS_LOG_FUNCTION(this);

        if (m_node->GetObject<PacketSocketFactory> () == 0) {
            PacketSocketHelper packetSocket;
            packetSocket.Install(m_node);
        }

        // Verify if port is valid
        if(dstPort<=0 || dstPort > 65535)
        {
            NS_FATAL_ERROR("Error: "<<dstPort<<" is not a valid port for UDP operations.");
        } else {

            

        // Create UDP socket
        m_udp_server_ip = Ipv4Address(udpServerIP.c_str());
        m_dst_port=dstPort;
        m_emulation = true;
        m_realtime = true;
        m_sock_emu = Socket::CreateSocket (m_node, TypeId::LookupByName ("ns3::UdpSocketFactory"));
        }

    }
    
    void
    RoadsideItsStation::SendHelloMessage() {
        NS_LOG_FUNCTION(this);

        std::string latlon = "lat"+std::to_string(m_position_lat)+"lon"+std::to_string(m_position_lon);
        Ptr<Packet> packet = Create<Packet> ((uint8_t*) latlon.c_str(), latlon.size());

        m_sock_emu->Send(packet);
    }

    void
    RoadsideItsStation::SendHelloMessageCont(double interval) {
        NS_LOG_FUNCTION(this);
        SendHelloMessage();
        Simulator::Schedule(Seconds(interval), &RoadsideItsStation::SendHelloMessageCont, this, interval);
    }

    void
    RoadsideItsStation::Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient) 
    {
        NS_LOG_FUNCTION(this);

        if (m_node->GetObject<PacketSocketFactory> () == 0) {
            PacketSocketHelper packetSocket;
            packetSocket.Install(m_node);
        }
        
        YansWifiPhyHelper wifiPhy;
        
        std::string id = sumoClient->GetStationId (this->GetNode ());
        unsigned long m_nodeID = std::stol(id.substr (4));

        // Create a new ETSI GeoNetworking socket
        m_sock=GeoNet::createGNPacketSocket(m_node);

        // If emulation is enabled, initialize UDP socket
        if(m_emulation == true) {
            if (m_sock_emu->Bind () == -1)
            {
                NS_FATAL_ERROR ("Failed to bind UDP socket");
            }
            if(m_sock_emu->Connect (InetSocketAddress(m_udp_server_ip,m_dst_port))!=0)
            {
                NS_FATAL_ERROR ("Error: cannot connect UDP socket.");
            }

        }

        // Configure Facilities headers in V2X socket //

        // Create a new Basic Service Container object
        m_bs_container = CreateObject<BSContainer>(m_nodeID, StationType_roadSideUnit,sumoClient,m_realtime,m_sock); 
        m_bs_container -> setupContainerRSU(true,false,false,false);
        

        // Set CA service
        Ptr<CABasicService> caService = m_bs_container->getCABasicService();
        
        // Get RSU position from SUMO and set it in the CA container
        libsumo::TraCIPosition rsuPosXY = sumoClient->TraCIAPI::poi.getPosition (id);
        libsumo::TraCIPosition rsuPosLonLat = sumoClient->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x,rsuPosXY.y);
        caService->setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);
        m_position_lat = rsuPosLonLat.y;
        m_position_lon = rsuPosLonLat.x;
        // Start capturing packets into a PCAP file
        wifiPhy.EnablePcap("rsu", m_node->GetDevice(0));

        // If emulation mode is enabled
        if(m_emulation == true) {
            // Configure retransmission of CAMs through emu socket
            caService->addCARxCallback (std::bind(&RoadsideItsStation::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));
            // Start sending Hello messages
            Simulator::Schedule(Seconds(0), &RoadsideItsStation::SendHelloMessageCont, this, 5);


        }
        
        // Output initialization message
        std::cout << "[t = " << Simulator::Now().GetSeconds() << " s] RSU node "<< nodeID << " initialized!" << std::endl;


    }

    void
    RoadsideItsStation::receiveCAM(asn1cpp::Seq<CAM> cam, Address from) {
        m_cams_received = m_cams_received+1;
        if(m_cams_received == 1) {
            // Send initialization packet
            SendHelloMessage();
        }
        // Process CAMs receive by other stations
        if(asn1cpp::getField(cam->header.stationId,StationID_t)!=m_nodeID) {
            // Encapsulate CAM into an UDP packet
            std::string encode_result = asn1cpp::uper::encode(cam);
                if(encode_result.size()<1)
                {
                    std::cout << "Error encapsulating CAM. " << std::endl;
                }
            
            Ptr<Packet> packet = Create<Packet> ((uint8_t*) encode_result.c_str(), encode_result.size());
            // Send through UDP socket
            m_sock_emu->Send(packet);
            
        }
    }
    
}





