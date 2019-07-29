/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2019 Regents of the University of California.
 *
 * This file is part of ndnSIM. See AUTHORS for complete list of ndnSIM authors and
 * contributors.
 *
 * ndnSIM is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * ndnSIM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ndnSIM, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

#ifndef NDNSIM_MODEL_DIRECTED_GEOCAST_STRATEGY_HPP
#define NDNSIM_MODEL_DIRECTED_GEOCAST_STRATEGY_HPP

#include "daemon/fw/strategy.hpp"

namespace nfd {
namespace fw {

/** \brief TBD
 */
class DirectedGeocastStrategy : public Strategy
{
public:
  // storage stuff inside pit entry

  explicit
  DirectedGeocastStrategy(Forwarder& forwarder, const Name& name = getStrategyName());

  static const Name&
  getStrategyName();

  void
  afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;

  void
  afterReceiveLoopedInterest(const FaceEndpoint& ingress, const Interest& interest,
                             pit::Entry& pitEntry) override;

private: // StrategyInfo
  /** \brief StrategyInfo on PIT entry
   */
  class PitInfo : public StrategyInfo
  {
  public:
    static constexpr int
    getTypeId()
    {
      return 9001;
    }

  public:
    std::map<FaceId, scheduler::ScopedEventId> queue;
  };
};

} // namespace fw
} // namespace nfd

#endif // NDNSIM_MODEL_DIRECTED_GEOCAST_STRATEGY_HPP
