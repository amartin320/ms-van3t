/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/
#include "camMonitor.h"

#include "ns3/CAM.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("camMonitor");

  NS_OBJECT_ENSURE_REGISTERED(camMonitor);

  TypeId
  camMonitor::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::camMonitor")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<camMonitor> ()
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&camMonitor::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&camMonitor::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&camMonitor::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&camMonitor::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&camMonitor::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
            "To enable/disable the transmission of CAM messages",
            BooleanValue(true),
            MakeBooleanAccessor (&camMonitor::m_send_cam),
            MakeBooleanChecker ());
        return tid;
  }

  camMonitor::camMonitor ()
  {
    NS_LOG_FUNCTION(this);

    m_client = nullptr;
    m_print_summary = true;
    m_already_print = false;
    m_cam_sent = 0;
  }

  camMonitor::~camMonitor ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  camMonitor::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  camMonitor::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    m_id = m_client->GetVehicleId (this->GetNode ());

    /* Create the socket for TX and RX */
    TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");

    /* Socket used to send CAMs and receive DENMs */
    m_socket = Socket::CreateSocket (GetNode (), tid);

    /* Bind the socket to local address */
    PacketSocketAddress local;
    local.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
    local.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetAddress ());
    local.SetProtocol (0x8947);
    if (m_socket->Bind (local) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind client socket");
    }

    /* Set the socket to broadcast */
    PacketSocketAddress remote;
    remote.SetSingleDevice (GetNode ()->GetDevice (0)->GetIfIndex ());
    remote.SetPhysicalAddress (GetNode ()->GetDevice (0)->GetBroadcast ());
    remote.SetProtocol (0x8947);

    m_socket->Connect(remote);

    /* Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService */
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();

    if(m_metric_supervisor!=nullptr)
    {
      m_geoNet->setMetricSupervisor(m_metric_supervisor);
    }

    m_btp->setGeoNet(m_geoNet);
    m_caService.setBTP(m_btp);

    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.addCARxCallback (std::bind(&camMonitor::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));
    m_caService.setStationProperties (std::stol(m_id.substr (3)), StationType_passengerCar);
    m_caService.setRealTime (m_real_time);
    VDP* traci_vdp = new VDPTraCI(m_client,m_id);

    m_caService.setVDP(traci_vdp);

    /* Create CSV file, if requested */
    if (!m_csv_name.empty ())
    {
      m_csv_ofstream.open (m_csv_name+"-"+m_id+".csv",std::ofstream::trunc);
      m_csv_ofstream << "messageID,originatingStationId,sequence,referenceTime,detectionTime,stationID" << std::endl;
    }

    /* Schedule CAM dissemination */
    if(m_send_cam == true)
    {
      std::srand(Simulator::Now().GetNanoSeconds ());
      double desync = ((double)std::rand()/RAND_MAX);
      m_caService.startCamDissemination(desync);
    }
  }

  void
  camMonitor::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_sendCamEvent);

    uint64_t cam_sent;
    cam_sent = m_caService.terminateDissemination ();

    if (!m_csv_name.empty ())
      m_csv_ofstream.close ();

    if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id
                << ",CAM-SENT:" << cam_sent
                << std::endl;
      m_already_print=true;
    }
  }

  void
  camMonitor::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }
  
  void
  camMonitor::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    /* Implement CAM strategy here */

    (void) cam;
    (void) from;

   // Free the received CAM data structure
//   ASN_STRUCT_FREE(asn_DEF_CAM,cam);
  }

  long
  camMonitor::compute_timestampIts ()
  {
    /* To get millisec since  2004-01-01T00:00:00:000Z */
    auto time = std::chrono::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

    long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
    return elapsed_since_2004;
  }
}





