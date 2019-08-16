/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2019,  Regents of the University of California,
 *                           Arizona Board of Regents,
 *                           Colorado State University,
 *                           University Pierre & Marie Curie, Sorbonne University,
 *                           Washington University in St. Louis,
 *                           Beijing Institute of Technology,
 *                           The University of Memphis.
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "directed-geocast-strategy.hpp"
#include "daemon/fw/algorithm.hpp"
#include "daemon/common/logger.hpp"
#include "daemon/common/global.hpp"

#include <ndn-cxx/lp/geo-tag.hpp>

#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include <tuple>
#include <sstream>
#include "math.h"
#include "ns3/vector.h"
#include <chrono>
namespace nfd {
namespace fw {

NFD_REGISTER_STRATEGY(DirectedGeocastStrategy);

NFD_LOG_INIT(DirectedGeocastStrategy);

DirectedGeocastStrategy::DirectedGeocastStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    NDN_THROW(std::invalid_argument("DirectedGeocastStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    NDN_THROW(std::invalid_argument(
      "DirectedGeocastStrategy does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));
}

const Name&
DirectedGeocastStrategy::getStrategyName()
{
  static Name strategyName("/localhost/nfd/strategy/directed-geocast/%FD%01");
  return strategyName;
}

void
DirectedGeocastStrategy::afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                                              const shared_ptr<pit::Entry>& pitEntry)
{
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();

  int nEligibleNextHops = 0;

  for (const auto& nexthop : nexthops) {
    Face& outFace = nexthop.getFace();

    if ((outFace.getId() == ingress.face.getId() && outFace.getLinkType() != ndn::nfd::LINK_TYPE_AD_HOC) ||
        wouldViolateScope(ingress.face, interest, outFace)) {
      continue;
    }

    if (outFace.getLinkType() != ndn::nfd::LINK_TYPE_AD_HOC) {
      // for non-ad hoc links, send interest as usual
      this->sendInterest(pitEntry, FaceEndpoint(outFace, 0), interest);
      NFD_LOG_DEBUG(interest << " from=" << ingress << " pitEntry-to=" << outFace.getId());
    }
    else {
      std::weak_ptr<pit::Entry> pitEntryWeakPtr = pitEntry;
      auto faceId = ingress.face.getId();

      // if transmission was already scheduled, ignore the interest

      PitInfo* pi = pitEntry->insertStrategyInfo<PitInfo>().first;
      if (pi->queue.find(faceId) != pi->queue.end()) {
        NFD_LOG_DEBUG(interest << " already scheduled pitEntry-to=" << outFace.getId());
        continue;
      }

      // calculate time to delay interest
      auto delay = calculateDelay(interest);
      if (delay > 0_s) {
        scheduler::ScopedEventId event = getScheduler().schedule(delay, [this, pitEntryWeakPtr,
                                                                         faceId, interest] {
            auto pitEntry = pitEntryWeakPtr.lock();
            auto outFace = getFaceTable().get(faceId);
            if (pitEntry == nullptr || outFace == nullptr) {
              // something bad happened to the PIT entry, nothing to process
              return;
            }

            this->sendInterest(pitEntry, FaceEndpoint(*outFace, 0), interest);
            NFD_LOG_DEBUG("delayed " << interest << " pitEntry-to=" << faceId);
          });

        // save `event` into pitEntry
        pi->queue.emplace(faceId, std::move(event));
      }
      else {
        this->sendInterest(pitEntry, FaceEndpoint(outFace, 0), interest);
        NFD_LOG_DEBUG("Could not determine to delay interest");
        NFD_LOG_DEBUG(interest << " from=" << ingress << " pitEntry-to=" << outFace.getId());
      }
    }

    ++nEligibleNextHops;
  }

  if (nEligibleNextHops == 0) {
    NFD_LOG_DEBUG(interest << " from=" << ingress << " noNextHop");

    // don't support NACKs (for now or ever)

    // lp::NackHeader nackHeader;
    // nackHeader.setReason(lp::NackReason::NO_ROUTE);
    // this->sendNack(pitEntry, ingress, nackHeader);

    this->rejectPendingInterest(pitEntry);
  }
}

void
DirectedGeocastStrategy::afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                                                    pit::Entry& pitEntry)
{
  // determine if interest needs to be cancelled or not

  PitInfo* pi = pitEntry.getStrategyInfo<PitInfo>();
  if (pi == nullptr) {
    NFD_LOG_DEBUG("Got looped interest, PitInfo is missing");
    return;
  }

  auto item = pi->queue.find(ingress.face.getId());
  if (item == pi->queue.end()) {
    NFD_LOG_DEBUG("Got looped interest, but no event was scheduled for the face");
    return;
  }

  if (shouldCancelTransmission(pitEntry, interest)) {
    item->second.cancel();

    // don't do anything to the PIT entry (let it expire as usual)
    NFD_LOG_DEBUG("Canceling transmission of " << interest << " via=" << ingress.face.getId());
  }
}

ndn::optional<ns3::Vector>
DirectedGeocastStrategy::getSelfPosition()
{
  auto node = ns3::NodeList::GetNode(ns3::Simulator::GetContext());
  if (node == nullptr) {
    return nullopt;
  }

  auto mobility = node->GetObject<ns3::MobilityModel>();
  if (mobility == nullptr) {
    return nullopt;
  }
  return mobility->GetPosition();

}

ndn::optional<ns3::Vector>
DirectedGeocastStrategy::extractPositionFromTag(const Interest& interest)
{
  auto tag = interest.getTag<ndn::lp::GeoTag>();
  if (tag == nullptr) {
  return nullopt;
  }

  auto pos = tag->getPos();
  return ns3::Vector(std::get<0>(pos), std::get<1>(pos), std::get<2>(pos));
}

time::nanoseconds
DirectedGeocastStrategy::calculateDelay(const Interest& interest)
{
  
  auto self = getSelfPosition(); 
  auto from = extractPositionFromTag(interest);
  time::nanoseconds(waitTime); 
  double distance = (self->GetLength()-from->GetLength()); 

  if (!self || !from) {
    NFD_LOG_DEBUG("self or from position is missing");
    return 0_s;
  }
  double minTime = 0.002; double maxDist = 150;
  if (distance < maxDist){
    NFD_LOG_DEBUG("self and from are within max limit");
    waitTime = time::duration_cast<time::nanoseconds>(time::duration<double>{(minTime * (maxDist-distance)/maxDist)});
    return waitTime;
  } 

  // TODO adding waitime calculation based on distance with correct format
  else{
  return waitTime = 10_ms;
  }
}

bool
DirectedGeocastStrategy::shouldCancelTransmission(const pit::Entry& oldPitEntry, const Interest& newInterest)
{
  auto self = getSelfPosition();
  auto oldFrom = extractPositionFromTag(oldPitEntry.getInterest());
  auto newFrom = extractPositionFromTag(newInterest);
  
  //distance calculation
  double distanceToLasthop = (self->GetLength() - newFrom->GetLength());
  double distanceToOldhop = (self->GetLength() - oldFrom->GetLength());
  double distanceBetweenLasthops = (newFrom->GetLength() - oldFrom->GetLength());

  //Angle calculation
  double Angle_rad = acos(pow(distanceToOldhop,2) + pow(distanceBetweenLasthops,2) - pow(distanceToLasthop,2) )/(2 * distanceToOldhop * distanceBetweenLasthops);
  double Angle_Deg = Angle_rad * 180 / 3.141592 ;

  // Projection Calculation
  double cosine_Angle_at_self = (pow (distanceToOldhop,2) + pow (distanceToLasthop,2) - pow (distanceBetweenLasthops,2))/(2 * distanceToOldhop * distanceToLasthop );
  double projection = distanceToLasthop * cosine_Angle_at_self;

  if (!self || !oldFrom || !newFrom) {
    NFD_LOG_DEBUG("self, oldFrom, or newFrom position is missing");
    return true;
  }
  if (Angle_Deg >= 90){
   NFD_LOG_DEBUG("Interest need not be cancelled");
   return false;
  }
  else if (projection > distanceToOldhop ){
  NFD_LOG_DEBUG("Interest need not be cancelled");
  return false;
  }
  return true;
}



} // namespace fw
} // namespace nfd
