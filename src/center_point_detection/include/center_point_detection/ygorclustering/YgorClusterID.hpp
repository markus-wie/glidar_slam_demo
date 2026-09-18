#ifndef YGOR_CLUSTERING_CLUSTERID_HPP
#define YGOR_CLUSTERING_CLUSTERID_HPP

// Copyright Haley Clark 2015.
//
///////////////////////////////////////////////////////////////////////////////
// This file is part of LibYgor.                                             //
//                                                                           //
// LibYgor is free software: you can redistribute it and/or modify           //
// it under the terms of the GNU General Public License as published by      //
// the Free Software Foundation, either version 3 of the License, or         //
// (at your option) any later version.                                       //
//                                                                           //
// LibYgor is distributed in the hope that it will be useful,                //
// but WITHOUT ANY WARRANTY; without even the implied warranty of            //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             //
// GNU General Public License for more details.                              //
//                                                                           //
// You should have received a copy of the GNU General Public License         //
// along with LibYgor.  If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <array>
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Utility class for (integral) cluster IDs. This class is essentially an integer (this->Raw)
//  wrapped with a bit of logic concerning getting the 'next' available ID and a few reserved
//  values.
//
//  At the moment, there are three types of IDs:
//    1. Unclassified -- which have not yet been clustered, or were not able to be clustered.
//    2. Noise        -- datum which are 'thought' to not belong to any clusters.
//    3. Regular IDs  -- a number indicating which cluster the datum belongs to.
//
template <class T>
class ClusterID
{
public:
  T Raw;

  static const auto Unclassified = std::numeric_limits<T>::max();
  static const auto Noise = std::numeric_limits<T>::max() - static_cast<T>(1);
  static const auto Cluster0 = std::numeric_limits<T>::min();  // Cluster1 = Cluster0 + 1, etc..

  constexpr ClusterID() : Raw(ClusterID<T>::Unclassified)
  {
  }
  explicit constexpr ClusterID(const ClusterID<T> & in) : Raw(in.Raw)
  {
  }
  explicit constexpr ClusterID(T in) : Raw(in)
  {
  }
  constexpr ClusterID<T> & operator=(const ClusterID<T> & rhs)
  {
    if (this != &rhs) {
      this->Raw = rhs.Raw;
    }
    return *this;
  }

  bool operator<(const ClusterID<T> & rhs) const
  {
    return this->Raw < rhs.Raw;
  }
  bool operator==(const ClusterID<T> & rhs) const
  {
    if (this == &rhs) {
      return true;
    }
    return this->Raw == rhs.Raw;
  }

  constexpr bool IsNoise() const
  {
    return this->Raw == ClusterID<T>::Noise;
  }

  constexpr bool IsUnclassified() const
  {
    return this->Raw == ClusterID<T>::Unclassified;
  }

  constexpr bool IsRegular() const
  {
    return (this->Raw != ClusterID<T>::Unclassified) && (this->Raw != ClusterID<T>::Noise);
  }

  bool IsNoise(T in) const
  {
    return in == ClusterID<T>::Noise;
  }

  bool IsUnclassified(T in) const
  {
    return in == ClusterID<T>::Unclassified;
  }

  bool IsRegular(T in) const
  {
    return (in != ClusterID<T>::Unclassified) && (in != ClusterID<T>::Noise);
  }

  ClusterID<T> NextValidClusterID() const
  {
    if (!this->IsRegular()) {
      return ClusterID<T>(ClusterID<T>::Cluster0);
    }

    ClusterID<T> CandidateID(this->Raw + static_cast<T>(1));
    if (!CandidateID.IsRegular()) {
      throw std::runtime_error(
        "Ran out of valid ClusterIDs. Use a type with larger range (e.g., uint64_t).");
    }
    return ClusterID<T>(CandidateID);
  }

  std::string ToText(void) const
  {
    std::string out;
    if (this->IsNoise()) {
      out = "Noise";
    } else if (this->IsUnclassified()) {
      out = "Unclassified";
    } else if (this->IsRegular()) {
      out = std::to_string(this->Raw);
    } else {
      throw std::runtime_error(
        "Programming error. This ClusterID is not Noise, Unclassified, or Regular. What is it?");
    }
    return out;
  }
};

#endif  // YGOR_CLUSTERING_CLUSTERID_HPP
