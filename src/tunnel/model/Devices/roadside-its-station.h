#ifndef ROADSIDE_ITS_STATION_H
#define ROADSIDE_ITS_STATION_H

#include <string>
#include "its-station.h"
#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/channel.h"    
#include "ns3/yans-wifi-helper.h"
#include "ns3/traci-module.h"
#include "ns3/BSContainer.h"



namespace ns3 {

class RoadsideItsStation : public ItsStation
{
  public:
    
    RoadsideItsStation ();
    RoadsideItsStation (std::string v2x_technology);

    virtual ~RoadsideItsStation ();

    void
    Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient);

  protected:

  private:
    
  };

} // namespace ns3

#endif /* ROADSIDE_ITS_STATION_H */