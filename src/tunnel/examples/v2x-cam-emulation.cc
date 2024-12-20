/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr> (initial IEEE 802.11p example)
 * Author: Junling Bu <linlinjavaer@gmail.com> (initial IEEE 802.11p example)
 * Author: Francesco Raviglione <francescorav.es483@gmail.com> (IEEE 802.11p simple CAM exchange application)
 *
 * This is a simple example of a ms-van3t V2V communication scenario configured with a single .cc file,
 * where vehicles exchange Cooperative Awareness Messages (CAMs) using the IEEE 802.11p standard.
 * The user can specify several parameters, including the priority (Access Category) for the transmission
 * of CAMs. BSContainers are used to simplify the configuration of the ETSI C-ITS stack of each vehicle.
 * In this scenario, all vehicles transmit CAMs according to the ETSI standards, and one vehicle (vehicle 3)
 * sends a heavy interfering traffic, without useful informative content, to simulate a congested channel.
 * Through the --interfering-userpriority option, the user can specify the Access Category (AC) for the
 * interfering traffic, which is broadcasted by vehicle 3.
 * When a new CAM is received by one of the vehicles, the callback "receiveCAM()" is called, and the stationID of
 * the received is available in "my_stationID".
 * Currently, the function just counts the total number of CAMs, but it can be customized to impement more complex
 * approaches.
 * As output, the simulation provides, thanks to the PRRSupervisor module, the average latency and PRR over the
 * whole simulation, and the average one-way latency for each vehicle up to vehicle 4.
 * The reported latency of vehicle 3 is expected to be 0 as it transmits interfering traffic (so, no ETSI-compliant
 * message is transmitted), that is not considered by the PRRSupervisor.
 */

#include "ns3/automotive-module.h"
#include "ns3/tunnel-module.h"
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include <iostream>
#include "ns3/MetricSupervisor.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/gn-utils.h"
#include "ns3/propagation-module.h"
#include "ns3/internet-module.h"
#include "ns3/emu-fd-net-device-helper.h"
#include "ns3/fd-net-device-module.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2Xcam-emu");
bool realtime = false;

int main (int argc, char *argv[])
{

  // Emulation parameters
  // Physical interface 
  std::string interface ("enx50a0300007da");
  // UDP parameters
  std::string udpServerIP = "192.168.1.254";
  std::string gwstr = "192.168.1.1";
  std::string subnet = "192.168.1.0";
  std::string netmask = "255.255.255.0";

  uint16_t dstPort=12345;
  Ipv4Address destAddr;
  Ipv4AddressHelper ipv4helper;

  // V2X parameters
  std::string v2x_tech = "802.11p"; // V2X technology to use (default: 802.11p)
  int txPower = 23.0; // IEEE 802.11p transmission power in dBm (default: 23 dBm)
  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // Default IEEE 802.11p data rate

  int numberOfVehicles; // Total number of vehicles, automatically filled in by reading the XML file
  int numberOfRSUs; // Total number of vehicles, automatically filled in by reading the XML file
  
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)

  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 100 seconds)
  float penetration_rate = 1.0; // Penetration rate of vehicles equipped with OBUs (default: 1.0 -- i.e, all vehicles)
  
  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string rsu_file = "stations.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";
  std::string sumo_fcdoutput_file_name = "fcd-output.xml";

  std::string sumo_additional_options = "--fcd-output " + sumo_fcdoutput_file_name + " --fcd-output.geo";

  bool sumo_gui = true; // By default, enable the SUMO GUI
  int first_vehicle_id;
  // Read the command line options
  CommandLine cmd (__FILE__);

  // Syntax to add new options: cmd.addValue (<option>,<brief description>,<destination variable>)
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("v2x-tech", "V2X technology to use (802.11p)", v2x_tech);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("real-time", "Enable real-time schedulling", realtime);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("rsu-file", "Name of the RSU file", rsu_file);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("sumo-gui", "Enable the SUMO GUI", sumo_gui);
  cmd.AddValue ("penetration-rate", "Configure vehicle penetration rate", penetration_rate);
  cmd.AddValue ("interface", "Name of the physical interface to send V2X messages to", interface);
  cmd.AddValue ("udp", "UDP server IP", udpServerIP);
  cmd.AddValue("dstPort", "Destination port of UDP server", dstPort);
  cmd.AddValue ("gateway", "Gateway at which the UDP/IP packets will be sent", gwstr);
  cmd.AddValue ("subnet", "Subnet which will be used to assign the IP addresses of emulated nodes (the .1 address is automatically excluded)", subnet);
  cmd.AddValue ("netmask", "Netmask of the network", netmask);


  cmd.Parse (argc, argv);

  if (verbose)
    {
      std::cout <<  "Verbose mode enabled" << std::endl;
      LogComponentEnable ("V2Xcam-emu", LOG_LEVEL_ALL);
      //LogComponentEnable ("CABasicService", LOG_LEVEL_ALL);
      //LogComponentEnable ("DENBasicService", LOG_LEVEL_ALL);
      //LogComponentEnable ("GeoNet", LOG_LEVEL_ALL);
      //LogComponentEnable ("btp", LOG_LEVEL_ALL);
      //LogComponentEnable ("TraceHelper", LOG_LEVEL_ALL);
      //LogComponentEnable ("PacketSocket", LOG_LEVEL_ALL);
      //LogComponentEnable ("Node", LOG_LEVEL_ALL);
      //LogComponentEnable ("NetDevice", LOG_LEVEL_ALL);
      //LogComponentEnable ("Socket", LOG_LEVEL_ALL);
      //LogComponentEnable ("TraciClient", LOG_LEVEL_ALL);
      LogComponentEnable ("ItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("VehicleItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("RoadsideItsStation", LOG_LEVEL_ALL);
      //LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
      //LogComponentEnable("ArpL3Protocol", LOG_LEVEL_ALL);
      //LogComponentEnable("ArpCache", LOG_LEVEL_ALL);
      //LogComponentEnable ("FdNetDevice", LOG_LEVEL_ALL);

    }

    

  // Enable real-time scheduler
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // Verify and parse emulation parameters
  if (udpServerIP != "")
    {
      destAddr = Ipv4Address(udpServerIP.c_str());

      if(dstPort<=0 || dstPort > 65535)
        {
          NS_FATAL_ERROR("Error: "<<dstPort<<" is not a valid port for UDP operations.");
        }

      std::cout<< "V2X Messages will be sent to:"<<destAddr<<":"<<dstPort << std::endl;

    }

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfVehicles = XML_rou_count_vehicles(rou_xml_file);
  first_vehicle_id = XML_rou_get_first_vehicle_id(rou_xml_file);
  std::cout << "Number of vehicles: " << numberOfVehicles << std::endl;

  /* Load RSU file*/
  std::string rsu_path = sumo_folder + rsu_file;
  std::ifstream rsu_file_stream (rsu_path.c_str());
  std::vector<std::tuple<std::string, float, float>> rsuData = XML_poli_count_stations(rsu_file_stream);
  numberOfRSUs = rsuData.size();
  std::cout << "Number of RSUs: " << numberOfRSUs << std::endl;
  xmlFreeDoc(rou_xml_file);/*** 1.1 Create containers for RSUs ***/
  xmlCleanupParser();

  // Check if there are enough nodes
  if(numberOfVehicles==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  // Create OBU nodes
  Ptr<VehicleItsStation> obus[numberOfVehicles];

  // Create RSU nodes
  Ptr<RoadsideItsStation> rsus[numberOfRSUs];

  // Create propagation channel
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  
  YansWifiPhyHelper wifiPhy;

  // Configure vehicle nodes
  
  for (int i = 0; i<numberOfVehicles; i++) {
    obus[i] = CreateObject<VehicleItsStation>("802.11p");
    obus[i]->SetChannel(channel);
    obus[i]->SetPhyMode(phyMode);
    obus[i]->SetTxPowerDbm(txPower);
    obus[i]->ConfigureRadio();

  }

  ipv4helper.SetBase (subnet.c_str(), netmask.c_str());
  ipv4helper.NewAddress (); // "Burn" the first IP as it may be assigned to a router in the local network

  // Configure RSU nodes 
  for (int i = 0; i<numberOfRSUs; i++) {
    rsus[i] = CreateObject<RoadsideItsStation>("802.11p");
    rsus[i]->SetChannel(channel);
    rsus[i]->SetPhyMode(phyMode);
    rsus[i]->SetTxPowerDbm(txPower);
    rsus[i]->ConfigureRadio();

    // Install IPv4 stack in RSUs
    InternetStackHelper internet;
    internet.SetIpv4StackInstall (true);
    internet.Install (rsus[i]->GetNode());
      
  }
  
  // Set up the TraCI interface and start SUMO with the default parameters
  // The simulation time step can be tuned by changing "SynchInterval"
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (penetration_rate));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (true));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("MaxVehicles", IntegerValue (numberOfVehicles));
  EmuFdNetDeviceHelper emuDev;
  emuDev.SetDeviceName (interface);
  emuDev.SetAttribute ("EncapsulationMode", StringValue ("Dix"));

  

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3));
      obus[nodeID-first_vehicle_id]->Initialize(nodeID, sumoClient, true);
      Ptr<Node> node = obus[nodeID-first_vehicle_id]->GetNode();
      return node;
    };

  SHUTDOWN_FCN shutdownWifiNode = [&] (Ptr<Node> exNode, std::string vehicleID)
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3));

      // Set position outside communication range 
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000000.0+(rand()%25),3200000.0+(rand()%25),250.0));;
      
      // Clean up BS container
      obus[nodeID-first_vehicle_id]->m_bs_container->cleanup();
    };

    sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

    NetDeviceContainer fdnetContainer;
    Ipv4InterfaceContainer ipv4ic;
    Ipv4Address gateway (gwstr.c_str());
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    // Initialize RSUs
    int i = 0;
    for (auto rsu : rsuData)
      {
        std::string id = std::get<0>(rsu);
        float x = std::get<1>(rsu);
        float y = std::get<2>(rsu);
        Ptr<Node> rsuNode = rsus[i]->GetNode();
        sumoClient->AddStation(id, x, y, 0.0, rsuNode);
        unsigned long nodeID = std::stol(id.substr (4));

        std::ostringstream mac;
        mac << "00:00:00:00:00:" << std::setfill('0') << std::setw(2) << nodeID+1;
        fdnetContainer = emuDev.Install(rsuNode);
        Ptr<FdNetDevice> dev = fdnetContainer.Get (0)->GetObject<FdNetDevice> ();
        dev->SetAddress (Mac48Address (mac.str().c_str ()));

        std::cout<<"MAC of node "<<nodeID<<": "<<mac.str()<<std::endl;        

        // Configure IP stack

        Ptr<Ipv4> ipv4 = rsuNode->GetObject<Ipv4> ();
        uint32_t iface = ipv4->AddInterface (dev);
        ipv4ic = ipv4helper.Assign (fdnetContainer);
        Ipv4InterfaceAddress address = Ipv4InterfaceAddress (ipv4ic.GetAddress (0,0), netmask.c_str());

        ipv4->AddAddress (iface, address);
        ipv4->SetMetric (iface, 1);
        ipv4->SetUp (iface);

        // Get Ipv4Interface object
        Ptr<Ipv4Interface> ifacePtr = ipv4->GetObject<Ipv4L3Protocol> ()->GetInterface (iface);
        
        Ptr<ArpCache> arp = CreateObject<ArpCache>();
        arp->SetAliveTimeout(Seconds(3600 * 24 * 365));
        ArpCache::Entry * entry = arp->Add(Ipv4Address (udpServerIP.c_str()));
        entry->SetMacAddress(Mac48Address("30:23:03:06:31:87"));
        entry->MarkPermanent();
        ifacePtr->SetArpCache(arp);

        Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
        staticRouting->SetDefaultRoute (gateway, iface);
        std::cout<<"IPv4 of node "<<nodeID<<": "<<ipv4ic.GetAddress (0,0)<<std::endl;
        
        rsus[i]->ConfigureEmulation(udpServerIP, dstPort);
        rsus[i]->Initialize(nodeID, sumoClient);
        
        //ApplicationContainer AppServer = CAM_MonitorHelper.Install (rsuNode);
        //AppServer.Start (Seconds (0.0));
        //AppServer.Stop (ns3::Seconds(simTime) - Seconds (0.1));
        ++i;
      }
    
    std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

    std::cout << "Starting simulation... " << std::endl;
    // Start simulation
    if (simTime > 0.0)
    {
      Simulator::Stop (Seconds(simTime));
    }

    Simulator::Run ();

    std::cout << "Run terminated..." << std::endl;


    Simulator::Destroy ();
  

  return 0;
}
