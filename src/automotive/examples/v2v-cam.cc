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
#include "ns3/automotive-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2Vcam");

// ******* DEFINE HERE ANY LOCAL GLOBAL VARIABLE, ACCESSIBLE FROM ANY FUNCTION IN THIS FILE *******
// Variables defined here should always be "static"
static int packet_count=0;
BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles
// ************************************************************************************************

// If you want to make a comparison with a received CAM MAC address, you can use:
// from == getGNAddress (0,Mac48Address(comparison_string)), where "comparison_string" is something like "00:00:00:00:00:09"

// Useful tip: you can get the latitude and longitude position of a given vehicle at any time with:
//   libsumo::TraCIPosition pos=<MobilityClient>->TraCIAPI::vehicle.getPosition(<string ID of the vehicle>);
//   pos=<MobilityClient>->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);
// <MobilityClient> should be the right Ptr<TraciClient> for the current vehicle, which can be retrieved from the
//   global Basic Services container with basicServices.get(my_stationID)->getTraCIclient ()
// The string ID of the vehicle should match the one in the XML file, i.e., for vehicle 7, the string id should be "veh7"
// After these two lines, pos.y will contain the latitude of the vehicle, while pos.x the longitude of the vehicle
void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
}


int main (int argc, char *argv[])
{

  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // Default IEEE 802.11p data rate
  int up=0;
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  int numberOfRSUs; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 23.0; // IEEE 802.11p transmission power in dBm (default: 23 dBm)
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

  // Read the command line options
  CommandLine cmd (__FILE__);

  // Syntax to add new options: cmd.addValue (<option>,<brief description>,<destination variable>)
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("rsu-file", "Name of the RSU file", rsu_file);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("sumo-gui", "Enable the SUMO GUI", sumo_gui);
  cmd.AddValue ("penetration-rate", "Configure vehicle penetration rate", penetration_rate);

  cmd.Parse (argc, argv);

  if (verbose)
    {
      std::cout <<  "Verbose mode enabled" << std::endl;
      LogComponentEnable ("V2Vcam", LOG_LEVEL_ALL);
      LogComponentEnable ("CABasicService", LOG_LEVEL_ALL);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_ALL);
      LogComponentEnable ("GeoNet", LOG_LEVEL_ALL);
      LogComponentEnable ("btp", LOG_LEVEL_ALL);
      LogComponentEnable ("TraceHelper", LOG_LEVEL_ALL);
      //LogComponentEnable ("PacketSocket", LOG_LEVEL_ALL);
      //LogComponentEnable ("Node", LOG_LEVEL_ALL);
      //LogComponentEnable ("NetDevice", LOG_LEVEL_ALL);
      //LogComponentEnable ("TraciClient", LOG_LEVEL_ALL);
      

    }

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);
  std::cout << "Number of vehicles: " << numberOfNodes << std::endl;

  /* Load RSU file*/
  std::string rsu_path = sumo_folder + rsu_file;
  std::ifstream rsu_file_stream (rsu_path.c_str());
  std::vector<std::tuple<std::string, float, float>> rsuData = XML_poli_count_stations(rsu_file_stream);
  numberOfRSUs = rsuData.size();
  std::cout << "Number of RSUs: " << numberOfRSUs << std::endl;
  xmlFreeDoc(rou_xml_file);/*** 1.1 Create containers for RSUs ***/
  xmlCleanupParser();

  // Check if there are enough nodes
  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  // Create OBU nodes
  NodeContainer obuNodes;
  obuNodes.Create (numberOfNodes);

  // Create RSU nodes
  NodeContainer rsuNodes;
  rsuNodes.Create (numberOfRSUs);

  PacketSocketHelper packetSocket;

  // Create propagation channel
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  YansWifiPhyHelper wifiPhy;

  // Create PHY and MAC for OBUs
  for (int i = 0; i<numberOfNodes; i++) {
    wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
    wifiPhy.SetChannel (channel);
    // ns-3 supports generating a pcap trace, to be later analyzed in Wireshark
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

    QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));

    NetDeviceContainer netDevice;
    netDevice = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes.Get(i));
    packetSocket.Install(obuNodes.Get(i));

  }

  // Create PHY and MAC for RSUs
  for (int i = 0; i<numberOfRSUs; i++) {
    wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
    wifiPhy.SetChannel (channel);
    // ns-3 supports generating a pcap trace, to be later analyzed in Wireshark
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

    QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));

    NetDeviceContainer netDevice;
    netDevice = wifi80211p.Install (wifiPhy, wifi80211pMac, rsuNodes.Get(i));
    packetSocket.Install(rsuNodes.Get(i));

  }
  

  if (verbose)
    {
      // wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging, only if verbose is true

    }

  // Set up the link between SUMO and ns-3, to make each node "mobile" (i.e., linking each ns-3 node to each moving vehicle in ns-3,
  // which corresponds to installing the network stack to each SUMO vehicle)
  MobilityHelper mobilityVehicles;
  mobilityVehicles.Install (obuNodes);
  MobilityHelper mobilityRSU;
  mobilityRSU.Install (rsuNodes);

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

  // Set up a Metricsupervisor
  // This module enables a trasparent and seamless collection of one-way latency (in ms) and PRR metrics
  Ptr<MetricSupervisor> metSup = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj(m_baseline_prr);
  metSup = &metSupObj;
  metSup->setTraCIClient(sumoClient);
  
  // Create RSU Apps

  camMonitorHelper CAM_MonitorHelper;
  CAM_MonitorHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  CAM_MonitorHelper.SetAttribute ("RealTime", BooleanValue(false));
  CAM_MonitorHelper.SetAttribute ("AggregateOutput", BooleanValue(true));
  CAM_MonitorHelper.SetAttribute ("CSV", StringValue("file.csv"));
  CAM_MonitorHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  int i = 0;
  for (auto rsu : rsuData)
    {
      std::string id = std::get<0>(rsu);
      float x = std::get<1>(rsu);
      float y = std::get<2>(rsu);
      Ptr<Node> rsuNode = rsuNodes.Get (i);
      sumoClient->AddStation(id, x, y, 0.0, rsuNode);
      ApplicationContainer AppServer = CAM_MonitorHelper.Install (rsuNode);
      AppServer.Start (Seconds (0.0));
      AppServer.Stop (ns3::Seconds(simTime) - Seconds (0.1));
      ++i;
    }
  
  // This function enables printing the current and average latency and PRR for each received packet
  // metSup->enablePRRVerboseOnStdout ();

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  // Important: what you write inside setupNewWifiNode() will be executed every time a new vehicle enters the simulation in SUMO
  // This kind of "std::function" is called lambda function, and it can access all variables outside its scope, thanks to the [&] capture
  // We setup here the ETSI stack for each vehicle (except the one generating interfering traffic), thanks to the BSContainer object
  // Furthermore, we schedule the transmission of interfering traffic for vehicle 3 only ("veh3")
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3));
      YansWifiPhyHelper wifiPhy;

      // Create a new ETSI GeoNetworking socket, thanks to the GeoNet::createGNPacketSocket() function, accepting as argument a pointer to the current node
      Ptr<Socket> sock;
      sock=GeoNet::createGNPacketSocket(obuNodes.Get(nodeID));
      // Set the proper AC, through the specified UP
      sock->SetPriority (up);
      
      // Create a new Basic Service Container object, which includes, for the current vehicle, both the ETSI CA Basic Service, for the transmission/reception
      // of periodic CAMs, and the ETSI DEN Basic Service, for the transmission/reception of event-based DENMs
      // An ETSI Basic Services container is a wrapper class to enable easy handling of both CAMs and DENMs
      // The station ID is set to be equal to the SUMO ID without "veh" (i.e., the station ID of "veh1" will be "1")
      Ptr<BSContainer> bs_container = CreateObject<BSContainer>(std::stol(vehicleID.substr(3)),StationType_passengerCar,sumoClient,false,sock);
      // Setup the PRRsupervisor inside the BSContainer, to make each vehicle collect latency and PRR metrics
      bs_container->linkMetricSupervisor(metSup);
      // This is needed just to simplify the whole application
      bs_container->disablePRRSupervisorForGNBeacons ();

      // Set the function which will be called every time a CAM is received, i.e., receiveCAM()
      bs_container->addCAMRxCallback (std::bind(&receiveCAM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      // Setup the new ETSI Basic Services container
      // The first parameter is true is you want to setup a CA Basic Service (for sending/receiving CAMs)
      // The second parameter should be true if you want to setup a DEN Basic Service (for sending/receiving DENMs)
      // The third parameter should be true if you want to setup a VRU Basic Service (for sending/receiving VAMs)
      bs_container->setupContainer(true,false,false,false);

      // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
      // retrieve the right BSContainer given a vehicle ID
      basicServices.add(bs_container);

      // Start transmitting CAMs
      // We randomize the instant in time in which the CAM dissemination is going to start
      // This simulates different startup times for the OBUs of the different vehicles, and
      // reduces the risk of multiple vehicles trying to send CAMs are the same time (causing more collisions);
      // "desync" is a value between 0 and 1 (seconds) after which the CAM dissemination should start
      std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
      double desync = ((double)std::rand()/RAND_MAX);
      bs_container->getCABasicService ()->startCamDissemination (desync);
      wifiPhy.EnablePcap("veh", obuNodes.Get(nodeID)->GetDevice(0));
      std::cout << "[t = " << Simulator::Now().GetSeconds() << " s] Node "<< nodeID << " initialized!" << std::endl;
      return obuNodes.Get(nodeID);
    };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
    {
      /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000000.0+(rand()%25),3200000.0+(rand()%25),250.0));

      // Turn off the Basic Services and the ETSI ITS-G5 stack for the vehicle
      // which has exited from the simulated scenario, and should be thus no longer considered
      // We need to get the right Ptr<BSContainer> based on the station ID (not the nodeID used
      // as index for the obuNodes), so we don't use "-1" to compute "intVehicleID" here
      unsigned long intVehicleID = std::stol(vehicleID.substr (3));

      Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
      bsc->cleanup();
    };


    sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

    // Set up RSUs
  // uint32_t i = 0;
  // for (auto rsu : rsuData)
  //   {
  //     YansWifiPhyHelper wifiPhy;
  //     // Add RSU to the simulation and to SUMO
  //     std::string id = std::get<0>(rsu);
  //     unsigned long nodeID = std::stol(id.substr (4,4));
  //     std::cout << "Adding RSU " << nodeID << " to the simulation..." << std::endl;
  //     float x = std::get<1>(rsu);
  //     float y = std::get<2>(rsu);
  //     Ptr<Node> rsuNode = rsuNodes.Get (i);
  //     sumoClient->AddStation(id, x, y, 0.0, rsuNode);

  //     // Install the application for the RSU
  //     TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  //     Ptr<Socket> socket = Socket::CreateSocket (rsuNode, tid);

  //     /* Bind the socket to local address */
  //     PacketSocketAddress local;
  //     local.SetSingleDevice (rsuNode->GetDevice (0)->GetIfIndex ());
  //     local.SetPhysicalAddress (rsuNode->GetDevice (0)->GetAddress ());
  //     local.SetProtocol (0x8947);
  //     if (socket->Bind (local) == -1)
  //     {
  //       NS_FATAL_ERROR ("Failed to bind server socket");
  //     }

  //     /* Set socket to broacdcast */
  //     PacketSocketAddress remote;
  //     remote.SetSingleDevice (rsuNode->GetDevice (0)->GetIfIndex ());
  //     remote.SetPhysicalAddress (rsuNode->GetDevice (0)->GetBroadcast ());
  //     remote.SetProtocol (0x8947);

  //     if (socket->Connect(remote) == -1)
  //     {
  //       NS_FATAL_ERROR ("Failed to connect server socket");
  //     } 
  //     /* Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService */
  //     Ptr<btp> bbtp = CreateObject <btp>();
  //     Ptr<GeoNet> geoNet = CreateObject <GeoNet>();
  //     CABasicService caService;
  //     bbtp->setGeoNet(geoNet);
  //     caService.setBTP(bbtp);
      

  //     /* Set callback and station properties in CABasicService */
  //     caService.setStationProperties (nodeID, StationType_roadSideUnit);
  //     caService.setSocketRx (socket);
  //     caService.setSocketTx (socket);
  //     caService.addCARxCallbackExtended (std::bind(&receiveCAM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      
  //     libsumo::TraCIPosition rsuPosXY = sumoClient->TraCIAPI::poi.getPosition (id);
  //     libsumo::TraCIPosition rsuPosLonLat = sumoClient->TraCIAPI::simulation.convertXYtoLonLat (rsuPosXY.x,rsuPosXY.y);

  //     caService.setFixedPositionRSU (rsuPosLonLat.y,rsuPosLonLat.x);
  //     VDP* traci_vdp = new VDPTraCI(sumoClient, id, true);

  //     caService.setVDP(traci_vdp);

  //     wifiPhy.EnablePcap("rsu", rsuNode->GetDevice(0));

  //     std::cout << "[t = " << Simulator::Now().GetSeconds() << " s] RSU "<< nodeID << " initialized!" << std::endl;
  //     ++i;

  //   }

    // Start simulation

    Simulator::Stop (Seconds(simTime));
    Simulator::Run ();
    
    // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
    std::cout << "Run terminated..." << std::endl;

    std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
    std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;

    // std::cout << "Average latency veh 1 (ms): " << metSup->getAverageLatency_vehicle (1) << std::endl;

    std::cout << "RX packet count: " << packet_count << std::endl;
    std::cout << "RX packet count (from PRR Supervisor): " << metSup->getNumberRx_overall () << std::endl;
    std::cout << "TX packet count (from PRR Supervisor): " << metSup->getNumberTx_overall () << std::endl;

    std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;
    

    Simulator::Destroy ();
  

  return 0;
}
