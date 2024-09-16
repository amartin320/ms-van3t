#ifndef ITS_STATION_H
#define ITS_STATION_H

#include <string>
#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/channel.h"    
#include "ns3/yans-wifi-helper.h"
#include "ns3/traci-module.h"
#include "ns3/BSContainer.h"



namespace ns3 {

class ItsStation : public Object
{
  public:
    
    ItsStation ();
    ItsStation (std::string v2x_technology);

    virtual ~ItsStation ();

    void Configure(void);
    void Configure80211p(std::string phyMode, double txPowerDbm);
    void ConfigureLTEV2X(void);
    void ConfigureNRV2X(void);

    void Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient);

    void SetChannel (Ptr<YansWifiChannel> channel);
    void SetV2XTechnology (std::string v2x_technology);
    void SetPhyMode (std::string phyMode);
    void SetTxPowerDbm (double txPowerDbm);

    Ptr<Node> GetNode(void);
    Ptr<Channel> GetChannel (void);
    std::string GetV2XTechnology (void) const;
    std::string GetPhyMode (void) const;
    double GetTxPowerDbm (void) const;

    Ptr<BSContainer> m_bs_container;

  protected:

    // Generic parameters
    bool m_debug; //!< Flag to enable debug output
    Ptr<Node> m_node;
  
    // Radio
    double m_txPowerDbm; //!< Transmission power in dBm
    std::string m_v2x_technology; //!< V2X Technology supported by the OBU

    // 802.11p parameters
    std::string m_phyMode; //!< 802.11p PHY mode
    Ptr<YansWifiChannel> m_channel_8011p;

    // ITS parameters

    unsigned long m_nodeID;
    Ptr<Socket> m_sock;

  private:

    
  };

} 

#endif // ITS_STATION_H
