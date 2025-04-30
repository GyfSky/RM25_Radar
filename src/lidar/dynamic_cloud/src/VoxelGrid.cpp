#include <open3d/t/geometry/RaycastingScene.h>
#include <tbb/parallel_for.h>
#include "VoxelGrid.h"

namespace upc_radar{
    inline double sqr_dis(const Eigen::Vector3d& a, const Eigen::Vector3d& b){
        return (a(0) - b(0)) * (a(0) - b(0)) + (a(1) - b(1)) * (a(1) - b(1)) + (a(2) - b(2)) * (a(2) - b(2));
    }

    void VoxelGrid::initialize(const VoxelGridParams &params) {
        /// @brief 初始化体素网格
        voxel_size = params.voxel_size;

        bbox = open3d::geometry::AxisAlignedBoundingBox(params.grid_min, params.grid_max);
        //得到体素网格的长宽高
        grid_size = ((bbox.GetMaxBound() - bbox.GetMinBound()) / voxel_size + Eigen::Vector3d::Ones()).cast<int>();
        //构建了一个三维的bool数组，大小由grid_size决定
        grid = std::vector<std::vector<std::vector<bool> > >(
            grid_size[0], std::vector<std::vector<bool> >(
                grid_size[1], std::vector<bool>(
                    grid_size[2], false
                )
            )
        );
        o3d_grid.voxel_size_ = voxel_size;
        //o3d_grid.origin_原点坐标  为什么要这么设置原点？将bool数组和VoxelGrid的原点对齐？
        o3d_grid.origin_ = bbox.GetMinBound() - Eigen::Vector3d::Ones() * voxel_size * 0.5;
    }

    void VoxelGrid::occupy_by_mesh(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh,
        Eigen::Vector3d lidar_pos, size_t dilate_size){
        /// @brief 利用模型占据体素网格

        open3d::t::geometry::RaycastingScene scene_r;
        //转换为tensor里的TriangleMesh，因为RaycastingScene是在tensor上实现的
        //向场景中添加mesh
        scene_r.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh));
        // double thres = Config["VoxelGrid"]["meshExpand"].as<double>();
        auto t2 = std::chrono::steady_clock::now();

        Eigen::MatrixXf Q(grid_size[0] * grid_size[1] * grid_size[2], 6);   // 查询光线
        std::vector<std::thread> threads;
        int step1=grid_size[0]/2, step2=grid_size[1]/2, step3=grid_size[2]/2;

        // for(int i=0;i<2;i++){
        //     for(int j=0;j<2;j++) {
        //         for(int k=0;k<2;k++) {
        //             threads.push_back(std::thread([i,j,k,&Q,step1,step2,step3,this,lidar_pos]() {
        //                 for(int x1=i*step1;x1<(i+1)*step1;x1++) {
        //                     for(int y1=j*step2;y1<(j+1)*step2;y1++) {
        //                         for(int z1=k*step3;z1<(k+1)*step3;z1++) {
        //                             Q.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(0) = lidar_pos.cast<float>()(0);
        //                             Q.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(1) = lidar_pos.cast<float>()(1);
        //                             Q.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(2) = lidar_pos.cast<float>()(2);
        //                             Eigen::Vector3d vec = get_grid_center({x1, y1, z1}) - lidar_pos;
        //                             Q.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(3) = vec.cast<float>()(0);
        //                             Q.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(4) = vec.cast<float>()(1);
        //                             Q.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(5) = vec.cast<float>()(2);
        //                         }
        //                     }
        //                 }
        //             }));
        //         }
        //     }
        // }
        // for(auto &t:threads){
        //     t.join();
        // }
        tbb::parallel_for(tbb::blocked_range<int>(0, int(grid_size[0])),
            [&](const tbb::blocked_range<int> &r) {
                for (int idx = r.begin(); idx < r.end(); ++idx) {
                    for(int y1=0;y1<grid_size[1];y1++) {
                        for(int z1=0;z1<grid_size[2];z1++) {
                            Q.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(0) = lidar_pos.cast<float>()(0);
                            Q.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(1) = lidar_pos.cast<float>()(1);
                            Q.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(2) = lidar_pos.cast<float>()(2);
                            Eigen::Vector3d vec = get_grid_center({idx, y1, z1}) - lidar_pos;
                            Q.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(3) = vec.cast<float>()(0);
                            Q.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(4) = vec.cast<float>()(1);
                            Q.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)(5) = vec.cast<float>()(2);
                        }
                    }

                }
        });
        // for (int i = 0; i < grid_size[0]; ++i) {
        //     for (int j = 0; j < grid_size[1]; ++j) {
        //         for (int k = 0; k < grid_size[2]; ++k) {
        //             Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(0) = lidar_pos.cast<float>()(0);
        //             Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(1) = lidar_pos.cast<float>()(1);
        //             Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(2) = lidar_pos.cast<float>()(2);
        //             Eigen::Vector3d vec = get_grid_center({i, j, k}) - lidar_pos;
        //             Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(3) = vec.cast<float>()(0);
        //             Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(4) = vec.cast<float>()(1);
        //             Q.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)(5) = vec.cast<float>()(2);
        //         }
        //     }
        // }

        auto t3 = std::chrono::steady_clock::now();
        auto dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        std::cout<<dur_time<<"ms1"<<std::endl;

        open3d::core::Tensor Q_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(Q);

        auto t4 = std::chrono::steady_clock::now();
        dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        std::cout<<dur_time<<"ms2"<<std::endl;

        // 计算每条由 雷达位置 到 网格中心 的线段是否与 mesh 相交, 相交则认为该网格被占据
        //如果雷达位置到网格中心的光线与 mesh 相交，则说明雷达光束照射不到此处，需要进行占据。
        //防止激光雷达散射值的影响。
        std::unordered_map<std::string, open3d::core::Tensor> rslt_r = scene_r.CastRays(Q_tensor);
        open3d::core::Tensor rslt_hit = rslt_r["t_hit"];
        // std::vector<std::thread> threads1;
        // for(int i=0;i<2;i++){
        //     for(int j=0;j<2;j++) {
        //         for(int k=0;k<2;k++) {
        //             threads1.push_back(std::thread([i,j,k,step1,step2,step3,this,rslt_hit,dilate_size]() {
        //                 for(int x1=i*step1;x1<(i+1)*step1;x1++) {
        //                     for(int y1=j*step2;y1<(j+1)*step2;y1++) {
        //                         for(int z1=k*step3;z1<(k+1)*step3;z1++) {
        //                             if (rslt_hit[x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1].Item<float>() < 1) {
        //                                 dilate_occupy(Eigen::Vector3i { x1, y1, z1 }, dilate_size, { 1, 0, 0 });
        //                             }
        //                         }
        //                     }
        //                 }
        //             }));
        //         }
        //     }
        // }
        // for(auto &t:threads1){
        //     t.join();
        // }

        // tbb::parallel_for(tbb::blocked_range<int>(0, int(grid_size[0])),
        //     [&](const tbb::blocked_range<int> &r) {
        //         for (int idx = r.begin(); idx < r.end(); ++idx) {
        //             for(int y1=0;y1<grid_size[1];y1++) {
        //                 for(int z1=0;z1<grid_size[2];z1++) {
        //                     if (rslt_hit[idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1].Item<float>() < 1) {
        //                         dilate_occupy(Eigen::Vector3i { idx, y1, z1 }, dilate_size, { 1, 0, 0 });
        //                     }
        //                 }
        //             }
        //
        //         }
        // });

        for (int i = 0; i < grid_size[0]; ++i) {
            for (int j = 0; j < grid_size[1]; ++j) {
                for (int k = 0; k < grid_size[2]; ++k) {
                    if (rslt_hit[i * grid_size[1] * grid_size[2] + j * grid_size[2] + k].Item<float>() < 1) {
                        dilate_occupy(Eigen::Vector3i { i, j, k }, dilate_size, { 1, 0, 0 });
                    }
                }
            }
        }
        auto t5 = std::chrono::steady_clock::now();
        dur_time = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
        std::cout<<dur_time<<"ms3"<<std::endl;
    }

    void VoxelGrid::occupy_by_mesh_filter(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh_f, double occupy_expand){
    
        open3d::t::geometry::RaycastingScene scene;
        scene.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_f));

        // 计算每个点到 mesh 的距离, 低于阈值的点被认为是在 mesh 内部而被占据
        Eigen::MatrixXf P(grid_size[0] * grid_size[1] * grid_size[2], 3); // 查询点集
        std::vector<std::thread> threads;
        int step1=grid_size[0]/2, step2=grid_size[1]/2, step3=grid_size[2]/2;

        // for(int i=0;i<2;i++){
        //     for(int j=0;j<2;j++) {
        //         for(int k=0;k<2;k++) {
        //             threads.push_back(std::thread([i,j,k,&P,step1,step2,step3,this]() {
        //                 for(int x1=i*step1;x1<(i+1)*step1;x1++) {
        //                     for(int y1=j*step2;y1<(j+1)*step2;y1++) {
        //                         for(int z1=k*step3;z1<(k+1)*step3;z1++) {
        //                             P.row(x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)
        //                                 = get_grid_center({ x1, y1, z1 }).cast<float>();
        //                         }
        //                     }
        //                 }
        //             }));
        //         }
        //     }
        // }
        // for(auto &t:threads){
        //     t.join();
        // }
        tbb::parallel_for(tbb::blocked_range<int>(0, int(grid_size[0])),
            [&](const tbb::blocked_range<int> &r) {
                for (int idx = r.begin(); idx < r.end(); ++idx) {
                    for(int y1=0;y1<grid_size[1];y1++) {
                        for(int z1=0;z1<grid_size[2];z1++) {
                            P.row(idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1)
                                        = get_grid_center({ idx, y1, z1 }).cast<float>();
                        }
                    }

                }
        });
        // for (int i = 0; i < grid_size[0]; ++i) {
        //     for (int j = 0; j < grid_size[1]; ++j) {
        //         for (int k = 0; k < grid_size[2]; ++k) {
        //             P.row(i * grid_size[1] * grid_size[2] + j * grid_size[2] + k)
        //                 = get_grid_center({ i, j, k }).cast<float>();
        //         }
        //     }
        // }
        open3d::core::Tensor P_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(P);
        open3d::core::Tensor rslt = scene.ComputeSignedDistance(P_tensor);
        // std::vector<std::thread> threads1;
        // for(int i=0;i<2;i++){
        //     for(int j=0;j<2;j++) {
        //         for(int k=0;k<2;k++) {
        //             threads1.push_back(std::thread([i,j,k,step1,step2,step3,this,rslt,occupy_expand]() {
        //                 for(int x1=i*step1;x1<(i+1)*step1;x1++) {
        //                     for(int y1=j*step2;y1<(j+1)*step2;y1++) {
        //                         for(int z1=k*step3;z1<(k+1)*step3;z1++) {
        //                             if (rslt[x1 * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1].Item<float>() < occupy_expand) {
        //                                 occupy(Eigen::Vector3i { x1, y1, z1 }, { 0, 0, 1 });
        //                             }
        //                         }
        //                     }
        //                 }
        //             }));
        //         }
        //     }
        // }
        // for(auto &t:threads1){
        //     t.join();
        // }
        // tbb::parallel_for(tbb::blocked_range<int>(0, int(grid_size[0])),
        //     [&](const tbb::blocked_range<int> &r) {
        //         for (int idx = r.begin(); idx < r.end(); ++idx) {
        //             for(int y1=0;y1<grid_size[1];y1++) {
        //                 for(int z1=0;z1<grid_size[2];z1++) {
        //                     if (rslt[idx * grid_size[1] * grid_size[2] + y1 * grid_size[2] + z1].Item<float>() < occupy_expand) {
        //                         occupy(Eigen::Vector3i { idx, y1, z1 }, { 0, 0, 1 });
        //                     }
        //                 }
        //             }
        //
        //         }
        // });
        for (int i = 0; i < grid_size[0]; ++i) {
            for (int j = 0; j < grid_size[1]; ++j) {
                for (int k = 0; k < grid_size[2]; ++k) {
                    if (rslt[i * grid_size[1] * grid_size[2] + j * grid_size[2] + k].Item<float>() < occupy_expand) {
                        occupy(Eigen::Vector3i { i, j, k }, { 0, 0, 1 });
                    }
                }
            }
        }
    }

}