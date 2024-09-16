
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


namespace ns3
{
    NS_LOG_COMPONENT_DEFINE("RoadsideItsStation");
    
    RoadsideItsStation::RoadsideItsStation () : ItsStation()
    {
        NS_LOG_FUNCTION(this);
    }

    RoadsideItsStation::~RoadsideItsStation ()
    {
        NS_LOG_FUNCTION(this);
    }

    RoadsideItsStation::RoadsideItsStation (std::string v2x_technology) : ItsStation(v2x_technology)
    {
        NS_LOG_FUNCTION(this);
    }

    void
    RoadsideItsStation::Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient) 
    {
        NS_LOG_FUNCTION(this);

        YansWifiPhyHelper wifiPhy;
        std::string id = sumoClient->GetStationId (this->GetNode ());
        unsigned long m_nodeID = std::stol(id.substr (4,4));

        /* Create the socket for TX and RX */
        TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");

        m_sock=GeoNet::createGNPacketSocket(m_node);

        // Create a new Basic Service Container object
        m_bs_container = CreateObject<BSContainer>(m_nodeID,StationType_roadSideUnit,sumoClient,false,m_sock); 
        m_bs_container -> setupContainerRSU(true,false,false,false);

        Ptr<CABasicService> caService = m_bs_container->getCABasicService();
        
        libsumo::TraCIPosition rsuPosXY = sumoClient->TraCIAPI::poi.getPosition (id);
        libsumo::TraCIPosition rsuPosLonLat = sumoClient->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x,rsuPosXY.y);
        caService->setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);

        // Start capturing packets into a PCAP file
        wifiPhy.EnablePcap("rsu", m_node->GetDevice(0));
        // Output initialization message
        std::cout << "[t = " << Simulator::Now().GetSeconds() << " s] RSU node "<< nodeID << " initialized!" << std::endl;


    }
    
}





