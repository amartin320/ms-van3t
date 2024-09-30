#include "ns3/tunnel-module.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("tunnel-test");

int main (int argc, char *argv[])
{
    LogComponentEnable ("obu", LOG_LEVEL_ALL);

    // Initialize propagation channel
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    Ptr<YansWifiChannel> channel = wifiChannel.Create ();
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");

    // Initialize OBU

    Ptr<VehicleItsStation> obu1 = CreateObject<VehicleItsStation> ("802.11p");
    obu1->SetChannel(channel);
    obu1->ConfigureRadio();
    
    return 0;
}
