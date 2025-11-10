#pragma once

/**
 * @file autonomousPathExample.hpp
 * @brief Example autonomous routines using the path follower
 * 
 * These are ready-to-use autonomous functions that demonstrate
 * different ways to use the path following system.
 */

// Follow the complete 4-curve path from JSON
void autonomousFollowPath();

// Navigate to multiple points in sequence (square pattern)
void autonomousMultiplePoints();

// Follow path with precision tuning (slow and accurate)
void autonomousPrecisionPath();

// Follow path with fast tuning (speed-focused)
void autonomousFastPath();

// Test just the first curve from the JSON
void autonomousTestFirstCurve();
