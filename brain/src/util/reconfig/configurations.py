
__all__ = [
    'DEFAULT',
    'DOOR',
    'DOOR_FWD',
    'DEADLOCK_EASY',
    'DEADLOCK_HARD'
]


DEFAULT = {
    'move_base': {
        'planner_frequency': 1
    },

    'move_base/DWAPlannerROS': {
        'sim_time': 5,
        'sim_granularity': 0.05,
        'path_distance_bias': 5,
        'goal_distance_bias': 10,
        'occdist_scale': 30,
        'forward_point_distance': 0.0,
        'vx_samples': 15,
        'vth_samples': 35,

        'oscillation_reset_dist': 0.2,
        'oscillation_reset_angle': 0.2
    },

    'move_base/global_costmap/inflation_layer': {
        'inflation_radius': 0.25,
    },

    'move_base/local_costmap': {
        'footprint_padding': 0.01,
        'resolution': 0.025
    },

    'move_base/local_costmap/inflation_layer': {
        'inflation_radius': 0.1,
    }
}

DOOR = {
    'move_base': {
        'planner_frequency': 0.5
    },

    'move_base/DWAPlannerROS': {
        'sim_time': 5,
        'sim_granularity': 0.025,
        'path_distance_bias': 2.5,
        'goal_distance_bias': 1,
        'occdist_scale': 5,
        'vx_samples': 20,
        'vth_samples': 40
    },

    'move_base/global_costmap/inflation_layer': {
        'inflation_radius': 0.1
    },

    'move_base/local_costmap': {
        'resolution': 0.025
    },

    'move_base/local_costmap/inflation_layer': {
        'inflation_radius': 0.025,
    }
}

DOOR_FWD = {
    'move_base': {
        'planner_frequency': 0.5
    },

    'move_base/DWAPlannerROS': {
        'sim_time': 4,
        'sim_granularity': 0.025,
        'path_distance_bias': 2.5,
        'goal_distance_bias': 1,
        'occdist_scale': 5,
        'forward_point_distance': 0.3,
        'vx_samples': 20,
        'vth_samples': 40
    },

    'move_base/global_costmap/inflation_layer': {
        'inflation_radius': 0.1
    },

    'move_base/local_costmap': {
        'resolution': 0.025
    },

    'move_base/local_costmap/inflation_layer': {
        'inflation_radius': 0.025,
    }
}


DEADLOCK_EASY = {
    'move_base': {
        'planner_frequency': 0.25
    },

    'move_base/DWAPlannerROS': {
        'sim_time': 4,
        'sim_granularity': 0.05,
        'path_distance_bias': 2.5,
        'goal_distance_bias': 1,
        'occdist_scale': 5,
        'oscillation_reset_dist': 0.5,
        'oscillation_reset_angle': 0.75,
        'vx_samples': 25
    },

    'move_base/local_costmap': {
        'footprint_padding': 0.005,
        'resolution': 0.025
    },

    'move_base/local_costmap/inflation_layer': {
        'inflation_radius': 0.05
    }
}

DEADLOCK_HARD = {
    'move_base': {
        'planner_frequency': 0.25
    },

    'move_base/DWAPlannerROS': {
        'sim_time': 4,
        'sim_granularity': 0.0125,
        'path_distance_bias': 2.5,
        'goal_distance_bias': 1,
        'occdist_scale': 5,
        'oscillation_reset_dist': 0.5,
        'oscillation_reset_angle': 0.75,
        'vx_samples': 25
    },

    'move_base/local_costmap': {
        'footprint_padding': 0.005,
        'resolution': 0.0125
    },

    'move_base/local_costmap/inflation_layer': {
        'inflation_radius': 0.0125
    }
}
