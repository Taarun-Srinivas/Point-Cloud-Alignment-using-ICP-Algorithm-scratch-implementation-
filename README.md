# Point-Cloud-Alignment-using-ICP-Algorithm-scratch-implementation

# Idea
Iterative Closest Point (ICP) is an alignment algorithm for merging two point clouds. It begins with an initial guess for the transformation between the point clouds. By finding corresponding points in each cloud based on proximity, ICP iteratively refines the transformation to minimize the distance between matched points. The process involves adjusting translation and rotation parameters until convergence or a set number of iterations is reached. ICP is widely applied in fields like computer vision and robotics to register and align 3D data, making it valuable for tasks such as object recognition and reconstruction.

# Required Packages
- Python
- Numpy
- Open3D (for visualization)

# Output Visualization
- results from open3d dataset
<div align="center">  

  <img src="https://github.com/Taarun-Srinivas/Point-Cloud-Alignment-using-ICP-Algorithm-scratch-implementation-/assets/52371207/d92b7e97-9c57-4c1d-a6ed-a108307ebc55"
     alt="initial alignment o3d example" width = 640 height = 480 />
  <img src="https://github.com/Taarun-Srinivas/Point-Cloud-Alignment-using-ICP-Algorithm-scratch-implementation-/assets/52371207/da0b6ce5-2a41-488f-af20-778d9b613fbc"
     alt="final alignment o3d example" width = 640 height = 480 />
</div>
<br></br>

- results from KITTI dataset
<div align="center">  

  <img src="https://github.com/Taarun-Srinivas/Point-Cloud-Alignment-using-ICP-Algorithm-scratch-implementation-/assets/52371207/731c6df8-e54e-4a91-b1c0-83318b1f03d7"
     alt="initial alignment pcd example" width = 640 height = 480 />
  <img src="https://github.com/Taarun-Srinivas/Point-Cloud-Alignment-using-ICP-Algorithm-scratch-implementation-/assets/52371207/b3f725bd-9c80-4b73-a643-bd3cc902b681"
     alt="final alignment pcd example" width = 640 height = 480 />
</div>
<br></br>
