/**
 * @brief  A class responsible for scoring the particle swarm based on
 *         observations from the vision system and handling particle injection.
 *
 * @author EJ Googins <egoogins@bowdoin.edu>
 * @date   February 2013
 * @author Josh Imhoff <joshimhoff13@gmail.com>
 * @date   June 2015
 */

#pragma once

#include "NBMath.h"
#include "LineSystem.h"
#include "LocStructs.h"
#include "FieldConstants.h"

#include <vector>
#include <list>
#include <map>
#include <boost/math/distributions.hpp>

namespace man {
namespace localization {

class VisionSystem
{
public:
    VisionSystem();
    ~VisionSystem();

    bool update(ParticleSet& particles,
                const messages::FieldLines& lines);
    int getNumObservations() const { return numObservations; }
    float getLowestError() const { return lowestError; }
    float getAvgError() const { return avgError; }
    float getWeightedAvgError() const { return weightedAvgError; }
    const std::list<ReconstructedLocation>& getInjections() { return injections; }

private:
    LineSystem* lineSystem;

    std::list<ReconstructedLocation> injections;
    int numObservations;
    float lowestError;
    float avgError;
    float weightedAvgError;
};

} // namespace localization
} // namespace man
