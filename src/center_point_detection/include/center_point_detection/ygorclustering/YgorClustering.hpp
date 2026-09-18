#ifndef YGOR_CLUSTERING_CLUSTERING_HPP
#define YGOR_CLUSTERING_CLUSTERING_HPP

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

#define BOOST_ALLOW_DEPRECATED_HEADERS
#include "boost/geometry/algorithms/within.hpp"
#include "boost/geometry/geometries/box.hpp"
#include "boost/geometry/geometries/point.hpp"
#include "boost/geometry/geometry.hpp"
#include "boost/geometry/index/parameters.hpp"
#include "boost/geometry/index/rtree.hpp"
#include "center_point_detection/ygorclustering/YgorClusterID.hpp"
#include "center_point_detection/ygorclustering/YgorClusteringDBSCAN.hpp"
#include "center_point_detection/ygorclustering/YgorClusteringDatum.hpp"
#include "center_point_detection/ygorclustering/YgorClusteringHelpers.hpp"

#endif  // YGOR_CLUSTERING_CLUSTERING_HPP
