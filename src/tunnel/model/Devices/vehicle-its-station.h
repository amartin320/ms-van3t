#ifndef VEHICLE_ITS_STATION_H
#define VEHICLE_ITS_STATION_H

#include <string>
#include "ns3/its-station.h"
#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/channel.h"    
#include "ns3/yans-wifi-helper.h"
#include "ns3/traci-module.h"
#include "ns3/BSContainer.h"



namespace ns3 {

class VehicleItsStation : public ItsStation
{
  public:
    
    VehicleItsStation ();
    VehicleItsStation (std::string v2x_technology);

    virtual ~VehicleItsStation ();

    void
    Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient, bool realtime);

  protected:

  private:
    
  };

} // namespace ns3

#endif /* VEHICLE_ITS_STATION_H */