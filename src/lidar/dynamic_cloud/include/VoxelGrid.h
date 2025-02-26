//
// Created by thesky on 25-2-6.
//

#include <vector>
#include <memory>

#include <Eigen/Core>
#include <open3d/Open3D.h>

namespace upc_radar{
    struct VoxelGridParams {
        Eigen::Vector3d grid_min;
        Eigen::Vector3d grid_max;
        double voxel_size;
    };

    class VoxelGrid {
        public:
        std::vector<std::vector<std::vector<bool>>> grid;
        std::vector<std::vector<std::vector<std::pair<size_t, size_t>>>> filter_grid;

        Eigen::Vector3i grid_size;
        double voxel_size;
        double voxel_size_f;
        open3d::geometry::AxisAlignedBoundingBox bbox;

        open3d::geometry::VoxelGrid o3d_grid;

        void initialize(const VoxelGridParams& params);
        void occupy_by_mesh(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh, Eigen::Vector3d lidar_pos, size_t dilate_size);
        void occupy_by_mesh_filter(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh_f, double occupy_expand);

        // 下面是实用函数
        inline void occupy(const Eigen::Vector3d& pt, Eigen::Vector3d color = { 0.5, 0.5, 0.5 }){
            /// @brief 将点 pt 所在的体素置为 true
            Eigen::Vector3d pt_grid = (pt - bbox.GetMinBound()) / voxel_size;
            Eigen::Vector3i pt_grid_int = pt_grid.cast<int>();
            occupy(pt_grid_int);
            o3d_grid.AddVoxel({ pt_grid_int, color });
        }
        inline void occupy(const Eigen::Vector3i& pt_grid_int, Eigen::Vector3d color = { 0.5, 0.5, 0.5 }){
            /// @brief 将点 pt 所在的体素置为 true
            if ((pt_grid_int.array() >= grid_size.array()).any() || (pt_grid_int.array() < 0).any())
                return;
            grid[pt_grid_int[0]][pt_grid_int[1]][pt_grid_int[2]] = true;
            o3d_grid.AddVoxel({ pt_grid_int, color });
        }
        inline void dilate_occupy(const Eigen::Vector3i& pti, size_t dilate_size, Eigen::Vector3d color = { 0.5, 0.5, 0.5 }){
            /// @brief 将点 pt 所在的体素置为 true 并扩张 dilate_size 个体素
            for (int di = -dilate_size; di <= static_cast<int>(dilate_size); ++di)
                for (int dj = -dilate_size; dj <= static_cast<int>(dilate_size); ++dj)
                    for (int dk = -dilate_size; dk <= static_cast<int>(dilate_size); ++dk) {
                        int ii = pti(0) + di, jj = pti(1) + dj, kk = pti(2) + dk;
                        if (ii < 0 || ii >= grid_size[0] || jj < 0 || jj >= grid_size[1] || kk < 0 || kk >= grid_size[2])
                            continue;
                        occupy(Eigen::Vector3i { ii, jj, kk }, color);
                    }
        }
        inline void dilate_occupy(const Eigen::Vector3d& pt, size_t dilate_size, Eigen::Vector3d color = { 0.5, 0.5, 0.5 }){
            /// @brief 将点 pt 所在的体素置为 true 并扩张 dilate_size 个体素
            Eigen::Vector3d pt_grid = (pt - bbox.GetMinBound()) / voxel_size;
            Eigen::Vector3i pt_grid_int = pt_grid.cast<int>();
            dilate_occupy(pt_grid_int, dilate_size, color);
        }
        inline bool is_occupied(const Eigen::Vector3d& pt) const{
            /// @brief 判断点 pt 所在的体素是否被占据
            if ((pt.array() >= bbox.GetMaxBound().array()).any() || (pt.array() < bbox.GetMinBound().array()).any())
                return true; // 超出范围的点默认为占据以过滤、
            Eigen::Vector3d pt_grid = (pt - bbox.GetMinBound()) / voxel_size;
            Eigen::Vector3i pt_grid_int = pt_grid.cast<int>();
            if ((pt_grid_int.array() >= grid_size.array()).any() || (pt_grid_int.array() < 0).any())
                return true; // 超出范围的点默认为占据以过滤
            return grid[pt_grid_int[0]][pt_grid_int[1]][pt_grid_int[2]];
        }
        inline Eigen::Vector3d get_grid_center(const Eigen::Vector3i& grid_idx) const{
            /// @brief 获取体素中心点坐calculate_zmap标
            return bbox.GetMinBound() + (grid_idx.cast<double>() + Eigen::Vector3d::Ones() * 0.5) * voxel_size;
        }
    };
}
