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
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/double.h"
#include <ctime>
#include <cstdlib>
namespace nfd {
namespace fw {

        NFD_REGISTER_STRATEGY(DirectedGeocastStrategy);

        NFD_LOG_INIT(DirectedGeocastStrategy);

        DirectedGeocastStrategy::DirectedGeocastStrategy(Forwarder &forwarder, const Name &name)
                : Strategy(forwarder) {
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

        const Name &
        DirectedGeocastStrategy::getStrategyName() {
            static Name strategyName("/localhost/nfd/strategy/directed-geocast/%FD%01");
            return strategyName;
        }

        void
        DirectedGeocastStrategy::afterReceiveInterest(const FaceEndpoint &ingress, const Interest &interest,
                                                      const shared_ptr <pit::Entry> &pitEntry) {
            const fib::Entry &fibEntry = this->lookupFib(*pitEntry);
            const fib::NextHopList &nexthops = fibEntry.getNextHops();

            int nEligibleNextHops = 0;

            for (const auto &nexthop : nexthops) {
                Face &outFace = nexthop.getFace();
                NDN_LOG_DEBUG(nexthop.getFace().getRemoteUri() << ", " << nexthop.getFace().getLocalUri());

                if ((outFace.getId() == ingress.face.getId() && outFace.getLinkType() != ndn::nfd::LINK_TYPE_AD_HOC) ||
                    wouldViolateScope(ingress.face, interest, outFace)) {
                    continue;
                }
                NFD_LOG_DEBUG("the link type is " << outFace.getLinkType());
                if (outFace.getLinkType() != ndn::nfd::LINK_TYPE_AD_HOC) {
                    // for non-ad hoc links, send interest as usual
                    this->sendInterest(pitEntry, FaceEndpoint(outFace, 0), interest);
                    NFD_LOG_DEBUG(interest << " from=" << ingress << " pitEntry-to=" << outFace.getId());
                } else {
                    std::weak_ptr<pit::Entry> pitEntryWeakPtr = pitEntry;
                    auto faceId = ingress.face.getId();

                    // if transmission was already scheduled, ignore the interest

                    PitInfo *pi = pitEntry->insertStrategyInfo<PitInfo>().first;
                    if (pi->queue.find(faceId) != pi->queue.end()) {
                        NFD_LOG_DEBUG(interest << " already scheduled pitEntry-to=" << outFace.getId());
                        continue;
                    }

                    // calculate time to delay interest
                    auto delay = calculateDelay(interest);
                    NFD_LOG_DEBUG("Delaying by " << delay);
                    if (delay > 0_s) {
                        scheduler::ScopedEventId event = getScheduler().schedule(delay, [this, pitEntryWeakPtr,
                                faceId, interest] {
                            auto pitEntry = pitEntryWeakPtr.lock();
                            auto outFace = getFaceTable().get(faceId);
                            if (pitEntry == nullptr || outFace == nullptr) {
                                // something bad happened to the PIT entry, nothing to process
                                return;
                            }

                            NFD_LOG_DEBUG("Sending out the delayed " << interest << " pitEntry-to=" << faceId);
                            this->sendInterest(pitEntry, FaceEndpoint(*outFace, 0), interest);
                        });

                        // save `event` into pitEntry
                        pi->queue.emplace(faceId, std::move(event));
                    } else {
                        NFD_LOG_DEBUG("Could not determine to delay interest, sending right away");
                        NFD_LOG_DEBUG(interest << " from=" << ingress << " pitEntry-to=" << outFace.getId());
                        this->sendInterest(pitEntry, FaceEndpoint(outFace, 0), interest);
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
        DirectedGeocastStrategy::afterReceiveLoopedInterest(const FaceEndpoint &ingress, const Interest &interest,
                                                            pit::Entry &pitEntry) {
            // determine if interest needs to be cancelled or not

            PitInfo *pi = pitEntry.getStrategyInfo<PitInfo>();
            if (pi == nullptr) {
                NFD_LOG_DEBUG("Got looped interest, PitInfo is missing");
                return;
            }
            auto item = pi->queue.find(ingress.face.getId());

            if (item == pi->queue.end()) {
                NFD_LOG_DEBUG("Got looped interest, but no event was scheduled for the face");
                return;
            }
            //NFD_LOG_DEBUG("Pitentry is" << pitEntry);
            if (shouldCancelTransmission(pitEntry, interest) == 1) {
                NFD_LOG_DEBUG(
                        "Canceling transmission of interest \n " << interest << "\n via=" << ingress.face.getId());
                item->second.cancel();

                //don't do anything to the PIT entry (let it expire as usual)

            }


        }

        ndn::optional <ns3::Vector>
        DirectedGeocastStrategy::getSelfPosition() {
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

        ndn::optional <ns3::Vector>
        DirectedGeocastStrategy::extractPositionFromTag(const Interest &interest) {
            auto tag = interest.getTag<ndn::lp::GeoTag>();
            NFD_LOG_DEBUG("the tag is " << tag);
            if (tag == nullptr) {
                return nullopt;
            }

            auto pos = tag->getPos();
            //NFD_LOG_DEBUG("the tagposition is " << pos);
            return ns3::Vector(std::get<0>(pos), std::get<1>(pos), std::get<2>(pos));
        }

        time::nanoseconds
        DirectedGeocastStrategy::calculateDelay(const Interest &interest) {
            auto self = getSelfPosition();
            auto from = extractPositionFromTag(interest);
            NFD_LOG_DEBUG("self " << self->GetLength());
            if (!self || !from) {
                NFD_LOG_DEBUG("self or from position is missing");
                return 0_s;
            }

            double distance = abs(self->GetLength() - from->GetLength());
           // std::srand(time(0));
            double minTime = 0.002;
            double maxDist = 1000;
            double maxTime = 2;
            if (distance < maxDist) {
                //auto waitTime = time::duration_cast<time::nanoseconds>(time::duration<double>{(minTime * (maxDist-distance)/maxDist)});
                double randomNumber = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX));
                ns3::RngSeedManager::SetSeed (3);
                ns3::SeedManager::SetRun (7);
                double min = 0.0;
                double max = 2.0;

                ns3::Ptr<ns3::UniformRandomVariable> x = ns3::CreateObject<ns3::UniformRandomVariable> ();
                x->SetAttribute ("Min", ns3::DoubleValue(min));
                x->SetAttribute ("Max", ns3::DoubleValue(max));
                auto myRandomNo = x->GetValue ();
                //float randomNumber = 0.003;
                NFD_LOG_DEBUG("the random number is " << myRandomNo);

                auto waitTime = time::duration_cast<time::nanoseconds>(time::duration < double >
                                                                       {((maxTime * (maxDist - distance) / maxDist) +
                                                                         minTime + myRandomNo)});
                // auto waitTime = ((maxTime * (maxDist-distance)/maxDist) + minTime);
                NFD_LOG_DEBUG("distance to last hop is " << distance << " meter");
                //NFD_LOG_DEBUG("distance to last hop is "<<distance<<" meter");
                NFD_LOG_DEBUG("self is at: " << self->GetLength() << " meter");
                NFD_LOG_DEBUG("from is at: " << from->GetLength() << " meter");
                NFD_LOG_DEBUG("self and from are within max limit hence delay is: " << waitTime);
                return waitTime;
            } else {
                NFD_LOG_DEBUG("Minimum Delay added is: 10ms ");
                return 10_ms;
            }
        }

        bool
        DirectedGeocastStrategy::shouldCancelTransmission(const pit::Entry &oldPitEntry, const Interest &newInterest) {
            NFD_LOG_DEBUG("Entered into Should cancel tranmission ");
            auto self = getSelfPosition();
            auto oldFrom = extractPositionFromTag(oldPitEntry.getInterest());
            auto newFrom = extractPositionFromTag(newInterest);

            if (!self || !oldFrom || !newFrom) {
                NFD_LOG_DEBUG("self, oldFrom, or newFrom position is missing");
                return false;
            }

            //oldFrom->GetLength() is the problem, it does not contain any value
            //NFD_LOG_DEBUG("self, oldform and newform are " << self->GetLength() << " " << newFrom->GetLength() << " " << oldFrom->GetLength());
            //distance calculation
            double distanceToLasthop = (self->GetLength() - newFrom->GetLength());
            NFD_LOG_DEBUG("distance to last hop is " << distanceToLasthop);
            double distanceToOldhop = (self->GetLength() - oldFrom->GetLength());
            NFD_LOG_DEBUG("distance to old hop is " << distanceToOldhop);
            double distanceBetweenLasthops = (newFrom->GetLength() - oldFrom->GetLength());
            NFD_LOG_DEBUG("distance between last hops is " << distanceBetweenLasthops);

            //Angle calculation
            double Angle_rad = acos(
                    (pow(distanceToOldhop, 2) + pow(distanceBetweenLasthops, 2) - pow(distanceToLasthop, 2)) /
                    (2 * distanceToOldhop * distanceBetweenLasthops));
            //double Angle_Deg = Angle_rad * 180 / 3.141592;
            double Angle_Deg = 91.00;
            NFD_LOG_DEBUG("angle is " << Angle_Deg);

            // Projection Calculation
            double cosine_Angle_at_self =
                    (pow(distanceToOldhop, 2) + pow(distanceToLasthop, 2) - pow(distanceBetweenLasthops, 2)) /
                    (2 * distanceToOldhop * distanceToLasthop);
            double projection = abs(distanceToLasthop * cosine_Angle_at_self);
            NFD_LOG_DEBUG("projection is " << projection);
            //bool state;

            if (Angle_Deg >= 90) {
                NFD_LOG_DEBUG("Interest need be cancelled");
                return true;
            } else if (projection > distanceToOldhop) {
                NFD_LOG_DEBUG("Interest need to be cancelled");
                return true;
            }

            return false;
        }


} // namespace fw
} // namespace nfd
