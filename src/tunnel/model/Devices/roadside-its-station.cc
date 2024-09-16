
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
    
}





