
#include "its-station.h"
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
    NS_LOG_COMPONENT_DEFINE("ItsStation");
    
    ItsStation::ItsStation ()
    {
        NS_LOG_FUNCTION(this);
        m_debug = false;
        m_node = CreateObject<Node>();
        m_v2x_technology = "802.11p";
        m_phyMode = "OfdmRate6MbpsBW10MHz";
        m_txPowerDbm = 23.0;
    }

    ItsStation::~ItsStation ()
    {
        NS_LOG_FUNCTION(this);
    }

    ItsStation::ItsStation (std::string v2x_technology)
    {
        NS_LOG_FUNCTION(this);
        m_debug = false;
        m_node = CreateObject<Node>();
        m_v2x_technology = v2x_technology;
        if(v2x_technology == "802.11p")
        {
            
            m_phyMode = "OfdmRate6MbpsBW10MHz";
            m_txPowerDbm = 23.0;
        }
    }

    // Initialization
    void
    ItsStation::Configure(void)
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
    ItsStation::Configure80211p(std::string phyMode, double txPowerDbm)
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

    // Setters
    void 
    ItsStation::SetChannel(Ptr<YansWifiChannel> channel)
    {
        NS_LOG_FUNCTION(this);
        m_channel_8011p = channel;
    }

    void
    ItsStation::SetV2XTechnology (std::string v2x_technology)
    {
        NS_LOG_FUNCTION(this);
        m_v2x_technology = v2x_technology;
    }

    void
    ItsStation::SetPhyMode (std::string phyMode)
    {
        NS_LOG_FUNCTION(this);
        m_phyMode = phyMode;
    }

    void
    ItsStation::SetTxPowerDbm (double txPowerDbm)
    {
        NS_LOG_FUNCTION(this);
        m_txPowerDbm = txPowerDbm;
    }
     
    // Getters  
    Ptr<Node>
    ItsStation::GetNode(void)
    {
        NS_LOG_FUNCTION(this);
        return m_node;
    }

    Ptr<Channel>
    ItsStation::GetChannel(void)
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
    ItsStation::GetV2XTechnology (void) const
    {
        NS_LOG_FUNCTION(this);
        return m_v2x_technology;
    }
    std::string
    ItsStation::GetPhyMode (void) const
    {
        NS_LOG_FUNCTION(this);
        return m_phyMode;
    }
    double
    ItsStation::GetTxPowerDbm (void) const
    {
        NS_LOG_FUNCTION(this);
        return m_txPowerDbm;
    }
    
    void
    ItsStation::Destroy (void) const
    {
        NS_LOG_FUNCTION(this);
        m_sock->Close();
        m_node->Dispose();
    }
}





