#include "ns3/lte-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/lte-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ndnSIM/helper/ndn-app-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"

#include "ns3/ndnSIM/model/directed-geocast-strategy.hpp"

#include <algorithm>
#include <vector>


NS_LOG_COMPONENT_DEFINE ("LteSlOutOfCovrgNDN");
namespace ns3{
int
main(int argc, char* argv[])
{
    CommandLine cmd;
    cmd.Parse (argc, argv);

    //Configure the UE for UE_SELECTED scenario
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue (16));
    Config::SetDefault ("ns3::LteUeMac::SlGrantSize", UintegerValue (5)); //The number of RBs allocated per UE for Sidelink
    Config::SetDefault ("ns3::LteUeMac::Ktrp", UintegerValue (1));
    Config::SetDefault ("ns3::LteUeMac::UseSetTrp", BooleanValue (true)); //use default Trp index of 0
  
    /* Although eNB and Ue bandwidth are not used in this scenario
     * Becasue UEs will use only the Sidelink to communicate, therefore, the EARFCN and the bandwidth are specified in the pool configuration. 
    * At this stage, the bottom two variables are initialized to be used later to configure the pathloss model and the Sidelink pool
    Set the frequency **/

    uint32_t ulEarfcn = 18100;  
    uint16_t ulBandwidth = 50;

  // Set error models
  Config::SetDefault ("ns3::LteSpectrumPhy::SlCtrlErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::LteSpectrumPhy::SlDataErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::LteSpectrumPhy::DropRbOnCollisionEnabled", BooleanValue (false));
    
    //Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23.0));

  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (2.0);

  //Create the helpers
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

  //Create and set the EPC helper
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);


  ////Create Sidelink helper and set lteHelper
  Ptr<LteSidelinkHelper> proseHelper = CreateObject<LteSidelinkHelper> ();
  proseHelper->SetLteHelper (lteHelper); 

  //Enable Sidelink
  lteHelper->SetAttribute ("UseSidelink", BooleanValue (true));

  //Set pathloss model
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::Cost231PropagationLossModel"));
  // channel model initialization
  lteHelper->Initialize ();

  // Since we are not installing eNB, we need to set the frequency attribute of pathloss model here
  double ulFreq = LteSpectrumValueHelper::GetCarrierFrequency (ulEarfcn);
  NS_LOG_LOGIC ("UL freq: " << ulFreq);
  Ptr<Object> uplinkPathlossModel = lteHelper->GetUplinkPathlossModel ();
  Ptr<PropagationLossModel> lossModel = uplinkPathlossModel->GetObject<PropagationLossModel> ();
  NS_ABORT_MSG_IF (lossModel == NULL, "No PathLossModel");
  bool ulFreqOk = uplinkPathlossModel->SetAttributeFailSafe ("Frequency", DoubleValue (ulFreq));
  if (!ulFreqOk)
    {
      NS_LOG_WARN ("UL propagation model does not have a Frequency attribute");
    }

  NS_LOG_INFO ("Deploying UE's...");

  //Create nodes (UEs)
  NodeContainer ueNodes;
  ueNodes.Create (4);
  NS_LOG_INFO ("UE 1 node id = [" << ueNodes.Get (0)->GetId () << "]");
  NS_LOG_INFO ("UE 2 node id = [" << ueNodes.Get (1)->GetId () << "]");
  NS_LOG_INFO ("UE 3 node id = [" << ueNodes.Get (2)->GetId () << "]");
  NS_LOG_INFO ("UE 4 node id = [" << ueNodes.Get (4)->GetId () << "]");

    //Fix Position of the nodes
    Ptr<ListPositionAllocator> positionAllocUe1 = CreateObject<ListPositionAllocator> ();
    positionAllocUe1->Add (Vector (0.0, 0.0, 1.5));
    Ptr<ListPositionAllocator> positionAllocUe2 = CreateObject<ListPositionAllocator> (); // distance at 20 m
    positionAllocUe2->Add (Vector (20.0, 0.0, 1.5));
    Ptr<ListPositionAllocator> positionAllocUe3 = CreateObject<ListPositionAllocator> (); // distance at 2 m
    positionAllocUe3->Add (Vector (2.0, 0.0, 1.5)); 
    Ptr<ListPositionAllocator> positionAllocUe4 = CreateObject<ListPositionAllocator> (); // distance at 100m
    positionAllocUe4->Add (Vector (100.0, 0.0, 1.5));

  //Install mobility

  MobilityHelper mobilityUe1;
  mobilityUe1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUe1.SetPositionAllocator (positionAllocUe1);
  mobilityUe1.Install (ueNodes.Get (0));

  MobilityHelper mobilityUe2;
  mobilityUe2.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUe2.SetPositionAllocator (positionAllocUe2);
  mobilityUe2.Install (ueNodes.Get (1));

  MobilityHelper mobilityUe3;
  mobilityUe3.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUe3.SetPositionAllocator (positionAllocUe1);
  mobilityUe3.Install (ueNodes.Get (2));

  MobilityHelper mobilityUe4;
  mobilityUe4.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUe4.SetPositionAllocator (positionAllocUe2);
  mobilityUe4.Install (ueNodes.Get (3));

  //Install LTE UE devices to the nodes
  NetDeviceContainer ueDevs = lteHelper->InstallUeDevice (ueNodes); 

  //Sidelink pre-configuration for the UEs to work without eNB
  Ptr<LteSlUeRrc> ueSidelinkConfiguration = CreateObject<LteSlUeRrc> ();
  ueSidelinkConfiguration->SetSlEnabled (true);

  LteRrcSap::SlPreconfiguration preconfiguration;

  preconfiguration.preconfigGeneral.carrierFreq = ulEarfcn;
  preconfiguration.preconfigGeneral.slBandwidth = ulBandwidth;
  preconfiguration.preconfigComm.nbPools = 1;

  LteSlPreconfigPoolFactory pfactory;

    //Control
  pfactory.SetControlPeriod ("sf40");
  pfactory.SetControlBitmap (0x00000000FF); //8 subframes for PSCCH
  pfactory.SetControlOffset (0);
  pfactory.SetControlPrbNum (22);
  pfactory.SetControlPrbStart (0);
  pfactory.SetControlPrbEnd (49);

  //Data
  pfactory.SetDataBitmap (0xFFFFFFFFFF);
  pfactory.SetDataOffset (8); //After 8 subframes of PSCCH
  pfactory.SetDataPrbNum (25);
  pfactory.SetDataPrbStart (0);
  pfactory.SetDataPrbEnd (49);

  preconfiguration.preconfigComm.pools[0] = pfactory.CreatePool ();

  ueSidelinkConfiguration->SetSlPreconfiguration (preconfiguration);
  lteHelper->InstallSidelinkConfiguration (ueDevs, ueSidelinkConfiguration);

  ///** Install NDN stack on all nodes **///

  ndn::StackHelper ndnHelper;
  ndnHelper.SetDefaultRoutes(true);
  ndnHelper.InstallAll();
  Ptr<LteSlTft> tft;
  //tft = Create<LteSlTft> (LteSlTft::BIDIRECTIONAL, groupAddress6, groupL2Address);

  //* Choosing forwarding strategy *//
  ndn::StrategyChoiceHelper::InstallAll("/","/localhost/nfd/strategy/best-route/DirectedGeocastStrategy");

  ///*** Configure applications ***///
  // Consumer
  ndn::AppHelper consumerHelper("ns3::ndn::ConsumerCbr");
  // Consumer will request /v2vsafety/0, /v2vsafety/1, ...
  consumerHelper.SetPrefix("/v2vsafety");
  consumerHelper.Install(ueNodes.Get(0));                        

  // Producer
  ndn::AppHelper producerHelper("ns3::ndn::Producer");
  // Producer will reply to all requests starting with /v2vsafety
  producerHelper.SetPrefix("/v2vsafety");
  producerHelper.Install(ueNodes.Get(1));
  producerHelper.Install(ueNodes.Get(2));
  producerHelper.Install(ueNodes.Get(3));

  //Set Sidelink bearers
  proseHelper->ActivateSidelinkBearer (slBearersActivationTime, ueDevs, tft);
  ///*** End of application configuration ***///

    NS_LOG_INFO ("Starting simulation...");
    Simulator::Stop(Seconds(20));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
}
int
main(int argc, char* argv[])
{
  return ns3::main(argc, argv);
}