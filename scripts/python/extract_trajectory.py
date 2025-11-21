import numpy as np
import argparse
import os
import converter as cvt

def parse_options():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_file', type=str, required=True, help='log file as (ts inliers tx ty tz rx ry rz [others])')
    parser.add_argument('--output_dir', type=str, default=".", help='directory of trajectory file saved')
    options = parser.parse_args()
    return options


def read_data(filename):
    return np.array([[float(x) for x in s.strip().split(' ')] for s in open(filename).readlines()])

def read_pose(options):
    log_file = options.log_file

    log_data = read_data(log_file)

    poses = log_data[:, [0, 2, 3, 4, 5, 6, 7]]
    poses[:, 4:] *= np.pi/180  # 转为弧度制

    return poses


def ypr2quat(poses):
    ts_and_pos = poses[:, :4]
    yprs = poses[:, 4:]
    
    Qs = np.array([cvt.YPR2Q(ypr, "ZYX", False) for ypr in yprs])

    vo_poses_as_quat = np.concatenate((ts_and_pos, Qs), axis=1)

    return vo_poses_as_quat


if __name__ == "__main__":
    options = parse_options()
    poses = read_pose(options)
    
    poses_tum_style = ypr2quat(poses)

    np.savetxt(os.path.join(options.output_dir, 'trajectory_tum.txt'), poses_tum_style, fmt='%.6f', delimiter=' ')