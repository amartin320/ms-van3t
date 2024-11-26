
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
#include "ns3/cv2x-module.h"
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/internet-module.h"
#include "ns3/config-store.h"



namespace ns3
{
    NS_LOG_COMPONENT_DEFINE("ItsStation");
    
    ItsStation::ItsStation ()
    {
        NS_LOG_FUNCTION(this);
        m_debug = false;
        m_node = CreateObject<Node>();
        m_v2x_technology = "802.11p";
        m_txPowerDbm = 23.0;
        SetDefault8011p();
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
            SetDefault8011p();
        }
        else if(v2x_technology == "LTE-V2X")
        {
            SetDefaultLTEV2X();
        }
    }

    void
    ItsStation::SetDefault8011p() {
        m_phyMode = "OfdmRate6MbpsBW10MHz";
    }

    void
    ItsStation::SetDefaultLTEV2X() {
        m_mcs = 20;
        m_probResourceKeep = 0.0;
        m_adjacencyPscchPssch = true;
        m_partialSensing = false;
        m_sizeSubchannel = 10;
        m_numSubchannel = 3;
        m_startRbSubchannel = 0;
        m_pRsvp = 20;
        m_t1 = 4;
        m_t2 = 100;
        m_slBandwidth;
    }
    // Initialization
    NetDeviceContainer
    ItsStation::ConfigureRadio(void)
    {
        NS_LOG_FUNCTION(this);
        // Configure Radio based on V2X technology

        if(m_v2x_technology == "802.11p"){
            Configure80211p(m_phyMode, m_txPowerDbm);
        }
        else if(m_v2x_technology == "LTE-V2X"){
            ConfigureLTEV2X(m_txPowerDbm, m_mcs);
        }
        else if (m_v2x_technology == "5G-V2X"){
            // Initialize 5G-V2X
        }

        // Install mobility in node
        MobilityHelper mobility;
        mobility.Install(m_node);
        return m_netDevice;
    }

    void
    ItsStation::Configure80211p(std::string phyMode, double txPowerDbm)
    {
        NS_LOG_FUNCTION(this);

        m_phyMode = phyMode;
        m_txPowerDbm = txPowerDbm;

        
        // Helpers
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

        

        m_netDevice = wifi80211p.Install (wifiPhy, wifi80211pMac, m_node);
        
    }

    void
    ItsStation::ConfigureLTEV2X(double txPowerDbm, uint32_t mcs)
    {
        NS_LOG_FUNCTION(this);


        Config::SetDefault ("ns3::cv2x_LteUePhy::TxPower", DoubleValue (txPowerDbm));
        Config::SetDefault ("ns3::cv2x_LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));

        /* Enable V2X communication on PHY layer */
        Config::SetDefault ("ns3::cv2x_LteUePhy::EnableV2x", BooleanValue (true));

        /* Disable power control: constant power*/
        Config::SetDefault ("ns3::cv2x_LteUePowerControl::Pcmax", DoubleValue (txPowerDbm));
        Config::SetDefault ("ns3::cv2x_LteUePowerControl::PsschTxPower", DoubleValue (txPowerDbm));
        Config::SetDefault ("ns3::cv2x_LteUePowerControl::PscchTxPower", DoubleValue (txPowerDbm));

        if (m_adjacencyPscchPssch)
        {
            m_slBandwidth = m_sizeSubchannel * m_numSubchannel;
        }
        else
        {
            m_slBandwidth = (m_sizeSubchannel+2) * m_numSubchannel;
        }

        /* Configure for UE selected */
        Config::SetDefault ("ns3::cv2x_LteUeMac::UlBandwidth", UintegerValue (m_slBandwidth));
        Config::SetDefault ("ns3::cv2x_LteUeMac::EnableAdjacencyPscchPssch", BooleanValue (m_adjacencyPscchPssch));
        Config::SetDefault ("ns3::cv2x_LteUeMac::EnablePartialSensing", BooleanValue (m_partialSensing));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SlGrantMcs", UintegerValue (m_mcs));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SlSubchannelSize", UintegerValue (m_sizeSubchannel));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SlSubchannelNum", UintegerValue (m_numSubchannel));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SlStartRbSubchannel", UintegerValue (m_startRbSubchannel));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SlPrsvp", UintegerValue (m_pRsvp));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SlProbResourceKeep", DoubleValue (m_probResourceKeep));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SelectionWindowT1", UintegerValue (m_t1));
        Config::SetDefault ("ns3::cv2x_LteUeMac::SelectionWindowT2", UintegerValue (m_t2));

        /*** 1. Create LTE objects   ***/
        
        Ptr<cv2x_PointToPointEpcHelper>  epcHelper = CreateObject<cv2x_PointToPointEpcHelper> ();
        Ptr<cv2x_LteHelper> lteHelper = CreateObject<cv2x_LteHelper> ();
        lteHelper->SetEpcHelper (epcHelper);

        // Disable eNBs for out-of-coverage modelling
        lteHelper->DisableNewEnbPhy();
        std::cout << "here!" << std::endl;
        /* V2X */
        Ptr<cv2x_LteV2xHelper> lteV2xHelper = CreateObject<cv2x_LteV2xHelper> ();
        lteV2xHelper->SetLteHelper (lteHelper);
        

        lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
        m_netDevice = lteHelper->InstallUeDevice (m_node);

        
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





