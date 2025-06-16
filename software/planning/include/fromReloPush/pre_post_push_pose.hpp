#ifndef PRE_POST_PUSH_POSE_HPP
#define PRE_POST_PUSH_POSE_HPP

namespace ReloPush {

    static float convertEulerRange_to_2pi(float angle) {
        if (angle < 0) {
            return angle + 2 * M_PI;
        } else {
            return angle;
        }
    }

    static float convertEulerRange_to_pi(float yaw) {
        if (yaw > M_PI) {
            yaw -= 2 * M_PI;
        }
        return yaw;
    }
    static State find_pre_push(State& goalState, float distance)
    {
        State outState(goalState);

        // Calculate the new x and y coordinates
        outState.x -= distance * cos(goalState.yaw);
        outState.y -= distance * sin(goalState.yaw);

        // change angle range
        outState.yaw = convertEulerRange_to_2pi(outState.yaw);

        return outState;
    }

    static State find_post_push(State& goalState, float distance)
    {
        State outState(goalState);

        // Calculate the new x and y coordinates
        outState.x += distance * cos(goalState.yaw);
        outState.y += distance * sin(goalState.yaw);

        return outState;
    }
}


#endif // PRE_POST_PUSH_POSE_HPP
