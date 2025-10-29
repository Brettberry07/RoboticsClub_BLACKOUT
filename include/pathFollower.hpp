#pragma once
#include "globals.hpp"
#include <cmath>
#include <array>

// Maximum number of curves and waypoints
constexpr int MAX_CURVES = 10;
constexpr int MAX_WAYPOINTS = 1000;

// Conversion constant: pixels to inches
// Field is 144 inches = 720 pixels
constexpr double PIXELS_TO_INCHES = 144.0 / 720.0; // = 0.2 inches per pixel

// Pure Pursuit Constants
struct PurePursuitConfig {
    double lookaheadDistance = 12.0;  // Distance in inches to look ahead on path
    double maxSpeed = 100.0;          // Maximum motor speed
    double minSpeed = 20.0;           // Minimum motor speed
    double turnDamping = 0.5;         // How much to reduce speed when turning
    double positionTolerance = 2.0;   // How close to end point to consider "arrived" (inches)
    double headingTolerance = 5.0;    // Heading tolerance in degrees
};

// 2D Point structure
struct Point {
    double x;  // in inches
    double y;  // in inches
    
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}
    
    // Distance to another point
    double distanceTo(const Point& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }
    
    // Angle to another point (in degrees)
    double angleTo(const Point& other) const {
        return atan2(other.y - y, other.x - x) * 180.0 / M_PI;
    }
};

// Bezier Curve structure matching the JSON format
struct BezierCurve {
    int curve;
    Point start;
    Point control1;
    Point middle;
    Point control2;
    Point end;
    bool reversed;
    
    // Sample a point along the bezier curve at parameter t (0.0 to 1.0)
    Point sampleAt(double t) const;
    
    // Get the tangent (heading) at parameter t
    double getTangentAt(double t) const;
    
    // Convert from pixels to inches
    void convertToInches();
};

// Path structure
struct Path {
    BezierCurve curves[MAX_CURVES];
    int curveCount;
    Point waypoints[MAX_WAYPOINTS];
    int waypointCount;
    
    Path() : curveCount(0), waypointCount(0) {}
    
    // Load path from JSON (currently embedded in paths.cpp)
    void loadFromJSON();
    
    // Generate waypoints from bezier curves
    void generateWaypoints(int pointsPerCurve = 50);
    
    // Get the closest point on the path to current position
    int getClosestPointIndex(const Point& currentPos) const;
    
    // Get lookahead point
    Point getLookaheadPoint(const Point& currentPos, int closestIndex, double lookaheadDist) const;
};

// Pure Pursuit Controller
class PurePursuitController {
private:
    PurePursuitConfig config;
    Path* currentPath;
    int lastClosestIndex;
    
public:
    PurePursuitController();
    
    // Set the path to follow
    void setPath(Path* path);
    
    // Set configuration
    void setConfig(const PurePursuitConfig& cfg);
    
    // Main pursuit function - returns motor speeds (left, right)
    void calculateMotorSpeeds(const Point& currentPos, double currentHeading, double& leftSpeed, double& rightSpeed);
    
    // Check if we've reached the end of the path
    bool hasReachedEnd(const Point& currentPos, double currentHeading) const;
    
    // Calculate curvature to a target point
    double calculateCurvature(const Point& currentPos, double currentHeading, const Point& targetPoint);
};

// Main path following function
void followPath(Path& path);

// Move to a specific point using pure pursuit
void moveTo(const Point& targetPoint, double targetHeading = -1);

// Helper function to get current robot position
Point getCurrentPosition();

// Helper function to normalize angle to [-180, 180]
double normalizeAngle(double angle);

// Calculate arc speeds for differential drive
void arcadeToTank(double forward, double turn, double& left, double& right);
