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
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/config-store.h"
#include "ns3/cv2x-module.h"



using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2Xcam");

bool realtime = false;
double probResourceKeep = 0.0;
bool adjacencyPscchPssch = true;
bool partialSensing = false;
uint32_t sizeSubchannel = 10;
uint32_t numSubchannel = 3;
uint32_t startRbSubchannel = 0;
uint32_t pRsvp = 20;
uint32_t t1 = 4;
uint32_t t2 = 100;
uint32_t slBandwidth;

void configureLTEV2XSidelink(NetDeviceContainer ueLteDevs, uint32_t numberOfNodes, Ptr<cv2x_LteHelper> lteHelper) {


  // Disable eNBs for out-of-coverage modelling
  lteHelper->DisableNewEnbPhy();

  /* V2X */
  Ptr<cv2x_LteV2xHelper> lteV2xHelper = CreateObject<cv2x_LteV2xHelper> ();
  lteV2xHelper->SetLteHelper (lteHelper);

  /* Configure eNBs' antenna parameters before deploying them. */
  lteHelper->SetEnbAntennaModelType ("ns3::cv2x_NistParabolic3dAntennaModel");
  lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
  Config::SetDefault ("ns3::cv2x_LteEnbNetDevice::UlEarfcn", StringValue ("54990")); // EARFCN 54990 -> 5855-5890-5925 MHz
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::cv2x_CniUrbanmicrocellPropagationLossModel"));
  NS_LOG_INFO("Antenna parameters set. Current EARFCN: 54990, current frequency: 5.89 GHz");

  lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
  
   /* Create sidelink groups */
  std::vector<NetDeviceContainer> txGroups;
  
  txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueLteDevs, numberOfNodes);

  /* Compute average number of receivers associated per transmitter and vice versa */
  std::map<uint32_t, uint32_t> txPerUeMap;
  std::map<uint32_t, uint32_t> groupsPerUe;
  std::vector<NetDeviceContainer>::iterator gIt;
  
  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
      {
          uint32_t numDevs = gIt->GetN();

          uint32_t nId;

          for(uint32_t i=1; i< numDevs; i++)
              {
                  nId = gIt->Get(i)->GetNode()->GetId();
                  txPerUeMap[nId]++;
              }
      }

  
  std::map<uint32_t, uint32_t>::iterator mIt;
  for(mIt=txPerUeMap.begin(); mIt != txPerUeMap.end(); mIt++)
      {
          groupsPerUe [mIt->second]++;
      }

  std::vector<uint32_t> groupL2Addresses;
  uint32_t groupL2Address = 0x00;
  std::vector<Ipv4Address> ipAddresses;
  Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask ("255.0.0.0"));
  Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
  NetDeviceContainer activeTxUes;


  for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
      {
          /* Create Sidelink bearers */
          NetDeviceContainer txUe ((*gIt).Get(0));
          activeTxUes.Add(txUe);
          NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
          Ptr<cv2x_LteSlTft> tft = Create<cv2x_LteSlTft> (cv2x_LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
          lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
          tft = Create<cv2x_LteSlTft> (cv2x_LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
          lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

          /* store and increment addresses */
          groupL2Addresses.push_back (groupL2Address);
          ipAddresses.push_back (clientRespondersAddress);
          groupL2Address++;
          clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
      }

  /* Creating sidelink configuration */
  Ptr<cv2x_LteUeRrcSl> ueSidelinkConfiguration = CreateObject<cv2x_LteUeRrcSl>();
  ueSidelinkConfiguration->SetSlEnabled(true);
  ueSidelinkConfiguration->SetV2xEnabled(true);

  cv2x_LteRrcSap::SlV2xPreconfiguration preconfiguration;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

  cv2x_SlV2xPreconfigPoolFactory pFactory;
  if (adjacencyPscchPssch)
    {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else
    {
        slBandwidth = (sizeSubchannel+2) * numSubchannel;
    }
  pFactory.SetHaveUeSelectedResourceConfig (true);
  pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
  pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
  pFactory.SetSizeSubchannel (sizeSubchannel);
  pFactory.SetNumSubchannel (numSubchannel);
  pFactory.SetStartRbSubchannel (startRbSubchannel);
  pFactory.SetStartRbPscchPool (0);
  pFactory.SetDataTxP0 (-4);
  pFactory.SetDataTxAlpha (0.9);

  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
  preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
  ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);

  lteHelper->InstallSidelinkV2xConfiguration (ueLteDevs, ueSidelinkConfiguration);
}

int main (int argc, char *argv[])
{


  std::string v2x_technology ("LTE-V2X"); // Default IEEE 802.11p data rate
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

  int first_vehicle_id;

  bool sumo_gui = true; // By default, enable the SUMO GUI
  
  // Read the command line options
  CommandLine cmd (__FILE__);

  // Syntax to add new options: cmd.addValue (<option>,<brief description>,<destination variable>)
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("v2x-technology", "V2X technology to use", v2x_technology);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("real-time", "Enable real-time schedulling", realtime);
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
      LogComponentEnable ("NetDevice", LOG_LEVEL_ALL);
      LogComponentEnable ("Socket", LOG_LEVEL_ALL);
      LogComponentEnable ("TraciClient", LOG_LEVEL_ALL);
      LogComponentEnable ("ItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("VehicleItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("RoadsideItsStation", LOG_LEVEL_ALL);
      LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
      LogComponentEnable ("cv2x_LteHelper", LOG_LEVEL_ALL);
      

    }

  if (realtime) {
    // Enable real-time scheduler
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
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
  std::cout << "First vehicle ID is: " << first_vehicle_id << std::endl;

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
  NetDeviceContainer vehicleNetDevices;
  InternetStackHelper internet;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  Ptr<cv2x_LteHelper> lteHelper;

  if(v2x_technology == "LTE-V2X") {
    Ptr<cv2x_PointToPointEpcHelper>  epcHelper = CreateObject<cv2x_PointToPointEpcHelper> ();
    lteHelper = CreateObject<cv2x_LteHelper> ();
    lteHelper->SetEpcHelper (epcHelper);
  }

  // Configure OBU nodes
  for (int i = 0; i<numberOfVehicles; i++) {
    obus[i] = CreateObject<VehicleItsStation>(v2x_technology);
    obus[i]->SetChannel(channel);
    obus[i]->SetTxPowerDbm(txPower);
    if(v2x_technology == "LTE-V2X") {
      obus[i]->SetLTEHelper(lteHelper);
      lteHelper->InstallUeDevice (obus[i]->GetNode());
      
    }
    vehicleNetDevices.Add(obus[i]->ConfigureRadio());
    /* Install the IP stack on the UE */
    internet.Install (obus[i]->GetNode());
  }

  // Configure RSU nodes 
  NetDeviceContainer rsuNetDevices;
  for (int i = 0; i<numberOfRSUs; i++) {
    rsus[i] = CreateObject<RoadsideItsStation>(v2x_technology);
    rsus[i]->SetChannel(channel);
    rsus[i]->SetTxPowerDbm(txPower);
    if(v2x_technology == "LTE-V2X") {
      rsus[i]->SetLTEHelper(lteHelper);
      lteHelper->InstallUeDevice (rsus[i]->GetNode());
      
    }
    rsuNetDevices.Add(rsus[i]->ConfigureRadio());
  }

  // Technology-specific global configs
  if(v2x_technology == "LTE-V2X") {
    configureLTEV2XSidelink(vehicleNetDevices, numberOfVehicles, lteHelper);
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
      obus[nodeID-first_vehicle_id]->Initialize(nodeID, sumoClient, realtime);
      Ptr<Node> node = obus[nodeID-first_vehicle_id]->GetNode();
      return node;
    };

  

  STARTUP_FCN setupNewLTEV2XNode = [&] (std::string vehicleID) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3));

      /* Assign IP address to UE */
      //Ipv4InterfaceContainer ueIpIface;
      //ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (m_netDevice));
      //Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (m_node->GetObject<Ipv4> ());
      //ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
      //NS_LOG_INFO("Node "<< m_node->GetId () << " has been assigned an IP address: " << m_node->GetObject<Ipv4> ()->GetAddress(1,0).GetLocal());

      obus[nodeID-first_vehicle_id]->Initialize(nodeID, sumoClient, realtime);
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
