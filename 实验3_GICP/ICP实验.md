1.生成一个人体模型SMPL，并生成深度序列(gen_depth_from_mesh.m)，即n个深度图

2.Pairwise ICP，把n个点云帧进行两两配准对齐到一个公共空间(通常是第一个深度图的空间)

3.Global ICP，在一个公共空间中求解n-1个位姿$[R_2,t_2,...,R_i,t_i,...,R_n,t_n]$，这里假设所有点云都对齐到第一个深度图(或点云)的空间。

