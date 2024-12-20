
#include "ns3/MetricSupervisor.h"

#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include "ns3/caBasicService.h"

#include "ns3/btp.h"
#include "ns3/traci-client.h"


namespace ns3 {

class camMonitor : public Application
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);

    camMonitor ();

    virtual ~camMonitor ();

    void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);
    void StopApplicationNow ();

  protected:
    virtual void DoDispose (void);

  private:

    CABasicService m_caService; //!< CA Basic Service object

    Ptr<btp> m_btp; //! BTP object
    Ptr<GeoNet> m_geoNet; //! GeoNetworking Object

    Ptr<Socket> m_socket; //!< Client socket

    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void TriggerCam(void);
    /**
     * @brief This function compute the milliseconds elapsed from 2004-01-01
    */
    long compute_timestampIts ();

    Ptr<TraciClient> m_client; //!< TraCI client
    std::string m_id; //!< vehicle id
    bool m_real_time; //!< To decide wheter to use realtime scheduler
    std::string m_csv_name; //!< CSV log file name
    std::ofstream m_csv_ofstream;
    bool m_print_summary; //!< To print a small summary when vehicle leaves the simulation
    bool m_already_print; //!< To avoid printing two summary
    Ipv4Address m_server_addr; //!< Remote addr

    EventId m_sendCamEvent; //!< Event to send the CAM

    /* Counters */
    int m_cam_sent;
    bool m_send_cam;
  };

} // namespace ns3
