/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 * Copyright (c) 2024 University of Stavanger
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
 * Author: Aitor Martin Rodriguez <aitor.martinrodriguez@uis.no> 

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


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2Xcam");


int main (int argc, char *argv[])
{


  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // Default IEEE 802.11p data rate
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)
  int numberOfVehicles; // Total number of vehicles, automatically filled in by reading the XML file
  int numberOfRSUs; // Total number of vehicles, automatically filled in by reading the XML file
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
      LogComponentEnable ("V2Xcam", LOG_LEVEL_ALL);
      LogComponentEnable ("CABasicService", LOG_LEVEL_ALL);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_ALL);
      LogComponentEnable ("GeoNet", LOG_LEVEL_ALL);
      LogComponentEnable ("btp", LOG_LEVEL_ALL);
      LogComponentEnable ("TraceHelper", LOG_LEVEL_ALL);
      LogComponentEnable ("PacketSocket", LOG_LEVEL_ALL);
      LogComponentEnable ("Node", LOG_LEVEL_ALL);
      //LogComponentEnable ("NetDevice", LOG_LEVEL_ALL);
      LogComponentEnable ("Socket", LOG_LEVEL_ALL);
      //LogComponentEnable ("TraciClient", LOG_LEVEL_ALL);
      LogComponentEnable ("ItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("VehicleItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("RoadsideItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("camMonitor", LOG_LEVEL_ALL);
      LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
      

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

  // Configure RSU nodes 
  for (int i = 0; i<numberOfRSUs; i++) {
    rsus[i] = CreateObject<RoadsideItsStation>("802.11p");
    rsus[i]->SetChannel(channel);
    rsus[i]->SetPhyMode(phyMode);
    rsus[i]->SetTxPowerDbm(txPower);
    rsus[i]->ConfigureRadio();
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


  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3));
      obus[nodeID-100]->Initialize(nodeID, sumoClient);
      Ptr<Node> node = obus[nodeID-100]->GetNode();
      return node;
    };

  SHUTDOWN_FCN shutdownWifiNode = [&] (Ptr<Node> exNode, std::string vehicleID)
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3));

      // Set position outside communication range 
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000000.0+(rand()%25),3200000.0+(rand()%25),250.0));;
      
      // Clean up BS container
      obus[nodeID-100]->m_bs_container->cleanup();
    };

    sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

    // Initialize RSUs
    int i = 0;
    for (auto rsu : rsuData)
      {
        std::string id = std::get<0>(rsu);
        float x = std::get<1>(rsu);
        float y = std::get<2>(rsu);
        Ptr<Node> rsuNode = rsus[i]->GetNode();
        sumoClient->AddStation(id, x, y, 0.0, rsuNode);
        unsigned long nodeID = std::stol(id.substr (4,4));
        rsus[i]->Initialize(nodeID, sumoClient);
        ++i;
      }
    

    // Start simulation

    Simulator::Stop (Seconds(simTime));
    Simulator::Run ();

    std::cout << "Run terminated..." << std::endl;


    Simulator::Destroy ();
  

  return 0;
}
