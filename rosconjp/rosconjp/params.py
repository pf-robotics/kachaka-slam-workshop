from dataclasses import dataclass


@dataclass
class MCLParams:
    odom_noise_a1: float
    odom_noise_a2: float
    odom_noise_a3: float
    odom_noise_a4: float
    resample_thresh: float
    effective_sample_size: int = 100
    num_particles: int = 100
    initial_noise_x: float = 0.1
    initial_noise_y: float = 0.1


@dataclass
class ICPParams:
    min_distance_to_match: float
    num_iter: int
    use_reciprocal: bool
    normal_search_radius: float
