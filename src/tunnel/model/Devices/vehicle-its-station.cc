
#include "vehicle-its-station.h"
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
    NS_LOG_COMPONENT_DEFINE("VehicleItsStation");
    
    VehicleItsStation::VehicleItsStation () : ItsStation()
    {
        NS_LOG_FUNCTION(this);
    }

    VehicleItsStation::~VehicleItsStation ()
    {
        NS_LOG_FUNCTION(this);
    }

    VehicleItsStation::VehicleItsStation (std::string v2x_technology) : ItsStation(v2x_technology)
    {
        NS_LOG_FUNCTION(this);
    }

    void 
    VehicleItsStation::Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient, bool realtime)
    {
        NS_LOG_FUNCTION(this);
        
        m_realtime = realtime;

        if (m_node->GetObject<PacketSocketFactory> () == 0) {
            PacketSocketHelper packetSocket;
            packetSocket.Install(m_node);
        }
        
        m_nodeID = nodeID;
        
        YansWifiPhyHelper wifiPhy;

        // Create a new ETSI GeoNetworking socket
        m_sock=GeoNet::createGNPacketSocket(m_node);
        

        // Create a new Basic Service Container object
        m_bs_container = CreateObject<BSContainer>(m_nodeID,StationType_passengerCar,sumoClient,m_realtime,m_sock);        

        // Enable basic services
        m_bs_container->setupContainer(true,true,false,false);

        // Start transmitting CAMs, randomizing start time
        std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
        double desync = ((double)std::rand()/RAND_MAX);
        m_bs_container->getCABasicService ()->startCamDissemination (desync);

        // Start capturing Tx/Rx packets into a PCAP file
        wifiPhy.EnablePcap("veh", m_node->GetDevice(0));

        // Output initialization message
        std::cout << "[t = " << Simulator::Now().GetSeconds() << " s] Vehicle node "<< nodeID << " initialized!" << std::endl;
    }
    
}





