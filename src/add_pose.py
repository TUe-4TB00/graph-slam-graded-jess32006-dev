
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    # TODO: Add X(4) odometry factor and initial estimate
    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), gtsam.Pose2(math.sqrt(2), math.sqrt(2), math.pi/2), ODOMETRY_NOISE))
    initial_estimate.insert(X(4), gtsam.Pose2(3.95+math.sqrt(2), math.sqrt(2)+0.05, math.pi/2+0.01))
    # TODO: Add the odometry factor between X(4) and X(5) to the graph (BetweenFactorPose2)
    # graph.add(gtsam.BetweenFactorPose2(X(4), X(5), gtsam.Pose2(0.0, 0.0, math.pi/4), ODOMETRY_NOISE))

    # TODO: Based on the odometry, find the initial estimate for the pose of X(5) and add it to the graph
    # initial_estimate.insert(X(5), gtsam.Pose2(5.4, 1.5, math.pi/2-0.05))
    return graph, initial_estimate