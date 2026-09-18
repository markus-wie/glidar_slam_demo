#pragma once

#include <string>
#include <vector>

#include "opencv2/core.hpp"

enum class LaneMode
{
  BOTH,
  LEFT,
  RIGHT
};

class Parameters
{
private:
  Parameters() = default;

public:
  static Parameters & getInstance()
  {
    static Parameters instance;
    return instance;
  }

  // Lf input vars
  float m_rayLength;
  int m_rayCount;
  std::vector<double> m_rayEllipticRatios;
  int m_debugPrintRays;
  float m_laneWidth;
  float m_maxAngle;
  float m_maxDistance;
  int m_maxPoints;
  LaneMode m_laneMode;

  // enable debug visuals
  bool m_debugPrint;

  // Region of interest constants
  int m_roiX;
  int m_roiY;

  bool m_drawIntersectionPoints;
  bool m_drawTranslatedPoints;

  float m_dbscanEps;
  int m_dbscanMinPoints;
  bool m_dbscanEnable;

  float m_vehicleWheelbase = 0.28f;
  float m_vehicleWheelSeparation = 0.26f;
  float m_vehicleCenterToCamOffset = 0.62f;
  float m_vehicleCamToWorldScaleFactor = 2.77f;

  Parameters(Parameters const &) = delete;
  void operator=(Parameters const &) = delete;
};
