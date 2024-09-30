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
#include "ns3/emu-fd-net-device-helper.h"
#include "ns3/internet-module.h"




namespace ns3 {

class RoadsideItsStation : public ItsStation
{
  public:
    
    RoadsideItsStation ();
    RoadsideItsStation (std::string v2x_technology);

    virtual ~RoadsideItsStation ();

    void
    ConfigureEmulation(std::string udpServerIP, uint16_t dstPort);

    void 
    SendHelloMessageCont(double interval);

    void
    SendHelloMessage();

    void
    Initialize(unsigned long nodeID, Ptr<TraciClient> sumoClient);

    void
    receiveCAM(asn1cpp::Seq<CAM> cam, Address from);

    void
    SetNetDeviceEmu(NetDeviceContainer * netDeviceContainer);

    NetDeviceContainer * GetNetDeviceEmu();

  protected:
    bool m_emulation;
    NetDeviceContainer * m_netDevice_emu;
    Ptr<Socket> m_sock_emu;
    Ipv4Address m_udp_server_ip;
    uint16_t m_dst_port;
    EmuFdNetDeviceHelper emuDev;
    double m_position_lat;
    double m_position_lon;
    int m_cams_received;

  private:
    
  };

} // namespace ns3

#endif /* ROADSIDE_ITS_STATION_H */