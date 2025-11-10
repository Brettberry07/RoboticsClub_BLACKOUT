#pragma once

// Test functions for the path following system

// Test following the complete path from JSON
void testPathFollower();

// Test moving to a specific point
void testMoveTo();

// Test moving to a point with a specific final heading
void testMoveToWithHeading();

// Test following a single bezier curve
void testIndividualCurve();

// Test with different tuning parameters
void tunePathFollower();
