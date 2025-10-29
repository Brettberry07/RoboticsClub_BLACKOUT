#include "pathFollower.hpp"
#include "globals.hpp"
#include <cmath>

// ===== BezierCurve Implementation =====

Point BezierCurve::sampleAt(double t) const {
    // Cubic Bezier curve formula.
    // B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
    // We have 5 control points, so we'll use a composite approach.
    
    double u = 1.0 - t;
    Point result;
    
    // First half uses start, control1, middle.
    if (t <= 0.5) {
        double localT = t * 2.0;  // Remap to [0, 1]
        double u2 = 1.0 - localT;
        
        result.x = u2 * u2 * start.x + 
                   2.0 * u2 * localT * control1.x + 
                   localT * localT * middle.x;
        result.y = u2 * u2 * start.y + 
                   2.0 * u2 * localT * control1.y + 
                   localT * localT * middle.y;
    }
    // Second half uses middle, control2, end.
    else {
        double localT = (t - 0.5) * 2.0;  // Remap to [0, 1]
        double u2 = 1.0 - localT;
        
        result.x = u2 * u2 * middle.x + 
                   2.0 * u2 * localT * control2.x + 
                   localT * localT * end.x;
        result.y = u2 * u2 * middle.y + 
                   2.0 * u2 * localT * control2.y + 
                   localT * localT * end.y;
    }
    
    return result;
}

double BezierCurve::getTangentAt(double t) const {
    // Get derivative for tangent.
    double dt = 0.001;
    Point p1 = sampleAt(std::max(0.0, t - dt));
    Point p2 = sampleAt(std::min(1.0, t + dt));
    
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    
    return atan2(dy, dx) * 180.0 / M_PI;
}

void BezierCurve::convertToInches() {
    start.x *= PIXELS_TO_INCHES;
    start.y *= PIXELS_TO_INCHES;
    control1.x *= PIXELS_TO_INCHES;
    control1.y *= PIXELS_TO_INCHES;
    middle.x *= PIXELS_TO_INCHES;
    middle.y *= PIXELS_TO_INCHES;
    control2.x *= PIXELS_TO_INCHES;
    control2.y *= PIXELS_TO_INCHES;
    end.x *= PIXELS_TO_INCHES;
    end.y *= PIXELS_TO_INCHES;
}

// ===== Path Implementation =====

void Path::loadFromJSON() {
    curveCount = 0;
    
    // Curve 1
    BezierCurve curve1;
    curve1.curve = 1;
    curve1.start = Point(116.13, 123.5);
    curve1.control1 = Point(122, 241);
    curve1.middle = Point(180.77, 240.69);
    curve1.control2 = Point(241, 240);
    curve1.end = Point(241, 359);
    curve1.reversed = false;
    curve1.convertToInches();
    curves[curveCount++] = curve1;
    
    // Curve 2
    BezierCurve curve2;
    curve2.curve = 2;
    curve2.start = Point(242.36, 357.81);
    curve2.control1 = Point(248.23, 475.31);
    curve2.middle = Point(183.88, 476.72);
    curve2.control2 = Point(120, 557);
    curve2.end = Point(124, 359);
    curve2.reversed = false;
    curve2.convertToInches();
    curves[curveCount++] = curve2;
    
    // Curve 3
    BezierCurve curve3;
    curve3.curve = 3;
    curve3.start = Point(122, 356);
    curve3.control1 = Point(141, 230);
    curve3.middle = Point(92.75, 221.5);
    curve3.control2 = Point(28, 201);
    curve3.end = Point(113, 123);
    curve3.reversed = false;
    curve3.convertToInches();
    curves[curveCount++] = curve3;
    
    // Curve 4
    BezierCurve curve4;
    curve4.curve = 4;
    curve4.start = Point(113, 127);
    curve4.control1 = Point(76, 368);
    curve4.middle = Point(174.13, 291.75);
    curve4.control2 = Point(293, 208);
    curve4.end = Point(173, 479);
    curve4.reversed = true;
    curve4.convertToInches();
    curves[curveCount++] = curve4;
}

void Path::generateWaypoints(int pointsPerCurve) {
    waypointCount = 0;
    
    for (int c = 0; c < curveCount && waypointCount < MAX_WAYPOINTS; c++) {
        const BezierCurve& curve = curves[c];
        for (int i = 0; i <= pointsPerCurve && waypointCount < MAX_WAYPOINTS; i++) {
            double t = static_cast<double>(i) / pointsPerCurve;
            Point p = curve.sampleAt(t);
            
            // If reversed, we'll handle direction in the controller
            waypoints[waypointCount++] = p;
        }
    }
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Generated %d waypoints", waypointCount);
}

int Path::getClosestPointIndex(const Point& currentPos) const {
    if (waypointCount == 0) return -1;
    
    int closestIndex = 0;
    double minDistance = currentPos.distanceTo(waypoints[0]);
    
    for (int i = 1; i < waypointCount; i++) {
        double distance = currentPos.distanceTo(waypoints[i]);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }
    
    return closestIndex;
}

Point Path::getLookaheadPoint(const Point& currentPos, int closestIndex, double lookaheadDist) const {
    if (waypointCount == 0 || closestIndex < 0) {
        return currentPos;
    }
    
    // Search forward along the path for a point at lookahead distance.
    for (int i = closestIndex; i < waypointCount; i++) {
        double distance = currentPos.distanceTo(waypoints[i]);
        
        if (distance >= lookaheadDist) {
            return waypoints[i];
        }
    }
    
    // If we didn't find a point, return the last waypoint.
    return waypoints[waypointCount - 1];
}

// ===== PurePursuitController Implementation =====

PurePursuitController::PurePursuitController() 
    : currentPath(nullptr), lastClosestIndex(0) {
}

void PurePursuitController::setPath(Path* path) {
    currentPath = path;
    lastClosestIndex = 0;
}

void PurePursuitController::setConfig(const PurePursuitConfig& cfg) {
    config = cfg;
}

void PurePursuitController::calculateMotorSpeeds(
    const Point& currentPos, double currentHeading, double& leftSpeed, double& rightSpeed) {
    
    if (!currentPath || currentPath->waypointCount == 0) {
        leftSpeed = 0.0;
        rightSpeed = 0.0;
        return;
    }
    
    // Find closest point on path.
    int closestIndex = currentPath->getClosestPointIndex(currentPos);
    
    // Ensure we don't go backwards.
    if (closestIndex < lastClosestIndex) {
        closestIndex = lastClosestIndex;
    }
    lastClosestIndex = closestIndex;
    
    // Get lookahead point.
    Point lookaheadPoint = currentPath->getLookaheadPoint(
        currentPos, closestIndex, config.lookaheadDistance);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Lookahead: (%.1f, %.1f)", 
                       lookaheadPoint.x, lookaheadPoint.y);
    
    // Calculate curvature to lookahead point.
    double curvature = calculateCurvature(currentPos, currentHeading, lookaheadPoint);
    
    // Calculate target heading.
    double targetHeading = currentPos.angleTo(lookaheadPoint);
    double headingError = normalizeAngle(targetHeading - currentHeading);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Heading error: %.1f", headingError);
    
    // Calculate base forward speed.
    double forwardSpeed = config.maxSpeed;
    
    // Reduce speed when turning sharply.
    double abserror = headingError < 0 ? -headingError : headingError;
    double turnFactor = 1.0 - (abserror / 90.0) * config.turnDamping;
    if (turnFactor < 0.3) turnFactor = 0.3;
    if (turnFactor > 1.0) turnFactor = 1.0;
    forwardSpeed *= turnFactor;
    
    // Clamp forward speed.
    if (forwardSpeed < config.minSpeed) forwardSpeed = config.minSpeed;
    if (forwardSpeed > config.maxSpeed) forwardSpeed = config.maxSpeed;
    
    // Calculate turn rate based on curvature.
    double turnRate = curvature * forwardSpeed;
    
    // Convert to tank drive (differential speeds).
    arcadeToTank(forwardSpeed, turnRate, leftSpeed, rightSpeed);
}

bool PurePursuitController::hasReachedEnd(const Point& currentPos, double currentHeading) const {
    if (!currentPath || currentPath->waypointCount == 0) {
        return true;
    }
    
    // Check if we're close to the final waypoint.
    Point endPoint = currentPath->waypoints[currentPath->waypointCount - 1];
    double distance = currentPos.distanceTo(endPoint);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Dist to end: %.1f", distance);
    
    return distance < config.positionTolerance;
}

double PurePursuitController::calculateCurvature(
    const Point& currentPos, double currentHeading, const Point& targetPoint) {
    
    // Transform target point to robot's coordinate frame.
    double dx = targetPoint.x - currentPos.x;
    double dy = targetPoint.y - currentPos.y;
    
    // Rotate to robot frame.
    double headingRad = currentHeading * M_PI / 180.0;
    double localX = dx * cos(-headingRad) - dy * sin(-headingRad);
    double localY = dx * sin(-headingRad) + dy * cos(-headingRad);
    
    // Calculate curvature: 2 * localY / (localX^2 + localY^2).
    double distance = sqrt(localX * localX + localY * localY);
    
    if (distance < 0.1) return 0.0;  // Avoid division by zero.
    
    double curvature = (2.0 * localY) / (distance * distance);
    
    return curvature;
}

// ===== Helper Functions =====

Point getCurrentPosition() {
    return Point(globalPos[0], globalPos[1]);
}

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void arcadeToTank(double forward, double turn, double& left, double& right) {
    left = forward + turn;
    right = forward - turn;
    
    // Find the maximum magnitude
    double maxMagnitude = left;
    if (maxMagnitude < 0) maxMagnitude = -maxMagnitude;
    double rightAbs = right < 0 ? -right : right;
    if (rightAbs > maxMagnitude) maxMagnitude = rightAbs;
    
    // Normalize if any value exceeds maximum speed
    if (maxMagnitude > 127.0) {
        left = (left / maxMagnitude) * 127.0;
        right = (right / maxMagnitude) * 127.0;
    }
}

// ===== Main Control Functions =====

void followPath(Path& path) {
    // Generate waypoints from Bezier curves.
    path.generateWaypoints(50);  // 50 points per curve
    
    // Create pure pursuit controller
    PurePursuitController controller;
    controller.setPath(&path);
    
    PurePursuitConfig config;
    config.lookaheadDistance = 12.0;   // 12 inches lookahead
    config.maxSpeed = 100.0;            // Adjust based on your robot.
    config.minSpeed = 20.0;
    config.positionTolerance = 3.0;
    controller.setConfig(config);
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Following path...");
    
    // Reset motor encoders.
    driveTrainMotors.tare_position();
    
    // Main control loop
    while (!controller.hasReachedEnd(getCurrentPosition(), globalHeading)) {
    // Update odometry.
        double leftTicks = leftChassis.get_position();
        double rightTicks = rightChassis.get_position();
        updateOdom(leftTicks, rightTicks);
        
    // Get current position and heading.
        Point currentPos = getCurrentPosition();
        double currentHeading = globalHeading;
        
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Pos: (%.1f, %.1f)", 
                           currentPos.x, currentPos.y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Heading: %.1f", currentHeading);
        
    // Calculate motor speeds.
        double leftSpeed, rightSpeed;
        controller.calculateMotorSpeeds(currentPos, currentHeading, leftSpeed, rightSpeed);
        
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "L: %.0f, R: %.0f", leftSpeed, rightSpeed);
        
    // Apply motor voltages (convert speed to voltage).
        leftChassis.move(leftSpeed);
        rightChassis.move(rightSpeed);
        
        pros::delay(20);  // 50Hz update rate
    }
    
    // Stop motors when finished.
    leftChassis.brake();
    rightChassis.brake();
    
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Path complete!");
    pros::delay(100);
}

void moveTo(const Point& targetPoint, double targetHeading) {
    // Create a simple path with just a straight line to the target.
    Path simplePath;
    
    // Create a Bezier curve from current position to target.
    Point currentPos = getCurrentPosition();
    
    BezierCurve curve;
    curve.start = currentPos;
    curve.end = targetPoint;
    
    // Create control points for a relatively straight path.
    Point midpoint((currentPos.x + targetPoint.x) / 2.0,
                   (currentPos.y + targetPoint.y) / 2.0);
    
    curve.control1 = Point(
        currentPos.x + (midpoint.x - currentPos.x) * 0.33,
        currentPos.y + (midpoint.y - currentPos.y) * 0.33
    );
    curve.middle = midpoint;
    curve.control2 = Point(
        midpoint.x + (targetPoint.x - midpoint.x) * 0.67,
        midpoint.y + (targetPoint.y - midpoint.y) * 0.67
    );
    curve.reversed = false;
    
    simplePath.curves[simplePath.curveCount++] = curve;
    
    // Follow the path.
    followPath(simplePath);
    
    // If a target heading was specified, turn to it.
    if (targetHeading >= 0) {
        angularPID(targetHeading);
    }
}
