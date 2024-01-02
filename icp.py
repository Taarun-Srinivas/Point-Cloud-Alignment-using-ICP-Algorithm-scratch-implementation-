import open3d as o3d
import copy
import numpy as np

demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud(r"C:\resources\Fall 2023\Robot Perception\HW2\kitti_frame1.pcd") #demo_icp_pcds.paths[0]
target = o3d.io.read_point_cloud(r"C:\resources\Fall 2023\Robot Perception\HW2\kitti_frame2.pcd") #demo_icp_pcds.paths[1]



# function to visualize the point clouds after the transformation has been applied
def draw_registration_result(source, target, transformation):
    """
    param: source - source point cloud
    param: target - target point cloud
    param: transformation - 4 X 4 homogeneous transformation matrix
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

initial_align = np.eye(4)
initial_align[:3,3] = np.array([0.3,0.5,10]).T
draw_registration_result(source, target, transformation=initial_align)

def Point_to_Point_ICP(source, target):
    prev_cost = 1e4
    curr_cost = 0
    final_trans = np.eye(4)
    trans_hist = []


    source_cpy = copy.deepcopy(source)
    target_cpy = copy.deepcopy(target)

    diff_cost = np.abs(prev_cost - curr_cost)
    while (diff_cost > 0.0001):
        source_points = []

        o3d.geometry.KDTreeSearchParamKNN(knn=7)
        source_tree = o3d.geometry.KDTreeFlann()
        source_tree.set_geometry(source_cpy)

        for i in range(len(np.array(target_cpy.points))):
            [_, id, _] = source_tree.search_knn_vector_3d(target_cpy.points[i], 7)
            source_points.append(source_cpy.points[id[0]])

        source_points = np.asarray(source_points)
        target_points = np.asarray(target_cpy.points)
        n = len(source_points)

        source_cm = np.transpose(source_points) - (np.sum(source_points, axis=0) / len(source_points)).reshape(3, 1)
        target_cm = np.transpose(target_points) - (np.sum(target_points, axis=0) / len(source_points)).reshape(3, 1)

        K = np.matmul(target_cm, source_cm.T)
        U, _, Vt = np.linalg.svd(K)
        R = np.matmul(U, Vt)

        source_points, target_points = np.transpose(source_points), np.transpose(target_points)
        T = (np.sum(target_points, axis=1) - np.matmul(R, np.sum(source_points, axis=1))) / n

        curr_cost = np.linalg.norm(target_cm - np.matmul(R, source_cm))
        diff_cost = np.abs(prev_cost - curr_cost)
        prev_cost = curr_cost
        print(diff_cost)

        H = np.eye(4)
        H[:3, :3] , H[:3, 3] = R, T
        final_trans = np.matmul(H, final_trans)
        # print(final_trans)
        trans_hist.append((diff_cost, final_trans))
        source_cpy.transform(H)




    return final_trans, trans_hist

final_trans, all_trans = Point_to_Point_ICP(source, target)
print(final_trans)
# print(all_trans)
draw_registration_result(source, target, final_trans)

