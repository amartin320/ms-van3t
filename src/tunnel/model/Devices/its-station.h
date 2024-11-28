#ifndef ITS_STATION_H
#define ITS_STATION_H

#include <string>
#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/channel.h"    
#include "ns3/yans-wifi-helper.h"
#include "ns3/traci-module.h"
#include "ns3/BSContainer.h"
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/cv2x-module.h"


namespace ns3 {

class ItsStation : public Object
{
  public:
    
    ItsStation ();
    ItsStation (std::string v2x_technology);

    virtual ~ItsStation ();

    void SetDefault8011p();
    void SetDefaultLTEV2X();
    void SetDefaultNRV2X();

    NetDeviceContainer ConfigureRadio(void);
    void Configure80211p(std::string phyMode, double txPowerDbm);
    void ConfigureLTEV2X(double txPowerDbm, uint32_t mcs);
    void ConfigureNRV2X(void);

    void Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient);

    void SetChannel (Ptr<YansWifiChannel> channel);
    void SetV2XTechnology (std::string v2x_technology);
    void SetPhyMode (std::string phyMode);
    void SetTxPowerDbm (double txPowerDbm);
    void SetLTEHelper (Ptr<cv2x_LteHelper> lteHelper);

    Ptr<Node> GetNode(void);
    Ptr<Channel> GetChannel (void);
    std::string GetV2XTechnology (void) const;
    std::string GetPhyMode (void) const;
    double GetTxPowerDbm (void) const;

    Ptr<BSContainer> m_bs_container;

    void Destroy (void) const;

  protected:

    // Generic parameters
    bool m_debug; //!< Flag to enable debug output
    Ptr<Node> m_node;
    NetDeviceContainer m_netDevice;
    bool m_realtime;
  
    // Radio
    double m_txPowerDbm; //!< Transmission power in dBm
    std::string m_v2x_technology; //!< V2X Technology supported by the OBU

    // 802.11p parameters
    std::string m_phyMode; //!< 802.11p PHY mode
    Ptr<YansWifiChannel> m_channel_8011p;

    // LTE-V2X parameters
    Ptr<cv2x_LteHelper> m_lteHelper;

    uint32_t m_mcs; // Modulation and coding scheme
    double m_probResourceKeep;          // Probability to select the previous resource again [0.0-0.8]
    // bool harqEnabled = false;               // Retransmission enabled (harq not available yet)
    bool m_adjacencyPscchPssch;        // Subchannelization scheme
    bool m_partialSensing;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t m_sizeSubchannel;           // Number of RBs per subchannel
    uint16_t m_numSubchannel;             // Number of subchannels per subframe
    uint16_t m_startRbSubchannel;         // Index of first RB corresponding to subchannelization
    uint16_t m_pRsvp;                    // Resource reservation interval
    uint16_t m_t1;                        // T1 value of selection window
    uint16_t m_t2;                      // T2 value of selection window
    uint16_t m_slBandwidth;                   // Sidelink bandwidth

    // ITS parameters

    unsigned long m_nodeID;
    Ptr<Socket> m_sock;

  private:

    
  };

} 

#endif // ITS_STATION_H
