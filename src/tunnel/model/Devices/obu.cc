
#include "obu.h"
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
    NS_LOG_COMPONENT_DEFINE("obu");

    obu::obu (Ptr<Node> node)
    {
        NS_LOG_FUNCTION(this);
        m_debug = false;
        m_node = node;
        m_v2x_technology = "802.11p";
        m_phyMode = "OfdmRate6MbpsBW10MHz";
        m_txPowerDbm = 23.0;
    }

    obu::~obu ()
    {
        NS_LOG_FUNCTION(this);
    }

    obu::obu (Ptr<Node> node, std::string v2x_technology)
    {
        NS_LOG_FUNCTION(this);
        m_debug = false;
        m_node = node;
        m_v2x_technology = v2x_technology;
        if(v2x_technology == "802.11p")
        {
            
            m_phyMode = "OfdmRate6MbpsBW10MHz";
            m_txPowerDbm = 23.0;
        }
    }

    // Initialization
    void
    obu::Configure(void)
    {
        NS_LOG_FUNCTION(this);

        if(m_v2x_technology == "802.11p")
        {
            Configure80211p(m_phyMode, m_txPowerDbm);
        }
        else if(m_v2x_technology == "LTE-V2X")
        {
            // Initialize LTE-V2X
        }
        else if (m_v2x_technology == "5G-V2X")
        {
            // Initialize 5G-V2X
        }

        MobilityHelper mobility;
        mobility.Install(m_node);
    }

    void
    obu::Configure80211p(std::string phyMode, double txPowerDbm)
    {
        NS_LOG_FUNCTION(this);

        m_phyMode = phyMode;
        m_txPowerDbm = txPowerDbm;

        
        // Helpers
        PacketSocketHelper packetSocket;
        YansWifiPhyHelper wifiPhy;
        Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
        QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
        

        // Initialize 802.11p

        wifiPhy.SetChannel (m_channel_8011p);
        wifiPhy.Set ("TxPowerStart", DoubleValue (m_txPowerDbm));
        wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txPowerDbm));
        wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

        wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode),
                                      "NonUnicastMode",StringValue (m_phyMode));
        if (m_debug == true) {
            wifi80211p.EnableLogComponents();
        }

        NetDeviceContainer netDevice;
        netDevice = wifi80211p.Install (wifiPhy, wifi80211pMac, m_node);
        packetSocket.Install(m_node);
        
    }

    void 
    obu::Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient)
    {

        m_nodeID = nodeID;
        YansWifiPhyHelper wifiPhy;

        // Create a new ETSI GeoNetworking socket
        m_sock=GeoNet::createGNPacketSocket(m_node);

        // Create a new Basic Service Container object
        m_bs_container = CreateObject<BSContainer>(m_nodeID,StationType_passengerCar,sumoClient,false,m_sock);

        // Enable basic services
        m_bs_container->setupContainer(true,true,false,false);


        // Start transmitting CAMs
        // We randomize the instant in time in which the CAM dissemination is going to start
        // This simulates different startup times for the OBUs of the different vehicles, and
        // reduces the risk of multiple vehicles trying to send CAMs are the same time (causing more collisions);
        // "desync" is a value between 0 and 1 (seconds) after which the CAM dissemination should start

        std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
        double desync = ((double)std::rand()/RAND_MAX);
        m_bs_container->getCABasicService ()->startCamDissemination (desync);

        // Start capturing packets into a PCAP file
        wifiPhy.EnablePcap("veh", m_node->GetDevice(0));

        // Output initialization message
        std::cout << "[t = " << Simulator::Now().GetSeconds() << " s] Vehicle node "<< nodeID << " initialized!" << std::endl;

    }

    // Setters
    void 
    obu::SetChannel(Ptr<YansWifiChannel> channel)
    {
        NS_LOG_FUNCTION(this);
        m_channel_8011p = channel;
    }

    void
    obu::SetV2XTechnology (std::string v2x_technology)
    {
        NS_LOG_FUNCTION(this);
        m_v2x_technology = v2x_technology;
    }

    void
    obu::SetPhyMode (std::string phyMode)
    {
        NS_LOG_FUNCTION(this);
        m_phyMode = phyMode;
    }

    void
    obu::SetTxPowerDbm (double txPowerDbm)
    {
        NS_LOG_FUNCTION(this);
        m_txPowerDbm = txPowerDbm;
    }
     
    // Getters  
    Ptr<Node>
    obu::GetNode(void)
    {
        NS_LOG_FUNCTION(this);
        return m_node;
    }

    Ptr<Channel>
    obu::GetChannel(void)
    {
        NS_LOG_FUNCTION(this);
        if(m_v2x_technology == "802.11p")
        {
            return m_channel_8011p;
        }
        else
        {
            return nullptr;
        }
    }

    std::string 
    obu::GetV2XTechnology (void) const
    {
        NS_LOG_FUNCTION(this);
        return m_v2x_technology;
    }
    std::string
    obu::GetPhyMode (void) const
    {
        NS_LOG_FUNCTION(this);
        return m_phyMode;
    }
    double
    obu::GetTxPowerDbm (void) const
    {
        NS_LOG_FUNCTION(this);
        return m_txPowerDbm;
    }

}





