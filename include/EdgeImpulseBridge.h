#pragma once

#include "GripperConfig.h"

// This adapter is intentionally optional. The data-collection firmware builds
// without an Edge Impulse library. Once a model is exported as an Arduino/C++
// library and placed at lib/edge_impulse, set kEnableEdgeImpulse to true in
// GripperConfig.h. The project then expects the normal Edge Impulse include
// layout: edge-impulse-sdk/classifier/ei_run_classifier.h.
#if __has_include(<edge-impulse-sdk/classifier/ei_run_classifier.h>)
#define GRIPPER_EDGE_IMPULSE_LIBRARY_FOUND 1
#else
#define GRIPPER_EDGE_IMPULSE_LIBRARY_FOUND 0
#endif

#if GRIPPER_ENABLE_EDGE_IMPULSE && !GRIPPER_EDGE_IMPULSE_LIBRARY_FOUND
#error "GRIPPER_ENABLE_EDGE_IMPULSE is set, but the Edge Impulse C++ library was not found."
#endif

#if GRIPPER_ENABLE_EDGE_IMPULSE
#include <edge-impulse-sdk/classifier/ei_run_classifier.h>
#include <edge-impulse-sdk/dsp/numpy.hpp>
#endif
