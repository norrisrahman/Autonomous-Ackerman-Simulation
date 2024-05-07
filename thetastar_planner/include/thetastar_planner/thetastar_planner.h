#ifndef _THETHASTAR_PLANNER_
#define _THETHASTAR_PLANNER_

#include <stdio.h>
#include <cstdlib> //abs
#include <cmath>   //sqrt
#include <string.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <unordered_set>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>

#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#define COSTTYPE unsigned char
#define MAX_COST 1.0e10

namespace thetastar_planner
{
    class Cell
    {
    public:
        Cell(int a, int b) : idx(a), f_cost(b) {}

        int idx;
        float f_cost;
    };

    struct greater
    {
        bool operator()(const Cell &a, const Cell &b) const
        {
            return a.f_cost > b.f_cost;
        }
    };

    struct lesser
    {
        bool operator()(const Cell &a, const Cell &b) const
        {
            return a.f_cost < b.f_cost;
        }
    };

    class ThetastarPlanner : public nav_core::BaseGlobalPlanner
    {
    private:
        /** costmap properties **/
        costmap_2d::Costmap2DROS *costmap_ros_; // http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html
        costmap_2d::Costmap2D *costmap_;        // http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

        std::vector<int> cmlocal;  // array of costs based on costmap
        std::string global_frame_; // goal and start coordinate frame
        int nx_;
        int ny_; // max no. of cells in x and y direction,
        int ns_; // total no. of cells
        double ox_;
        double oy_;  // origin of cosmap
        double res_; // resolution of costmap

        bool initialized_;
        bool allow_unknown_;
        bool publish_potential_;
        int publish_scale_;
        int neutral_cost_;
        double planner_window_x_, planner_window_y_, default_tolerance_;
        int cheapthreshold;
        double cost_factor_;
        bool allow_obs_plan_through;
        double obs_cost_ratio;
        bool get_grad_path_;

        float *gradx_, *grady_; /**< gradient arrays, size of potential array */
        float pathStep_;        /**< step size for following gradient */

        // Data structures
        std::vector<Cell> openList; // open list containing cell idx and f cost
        std::unordered_set<int> visited;
        std::vector<double> g_costs; // TODO: change this to a pointer too?
        std::vector<int> parents;
        float *f_costs;

        /** time **/
        int counter;
        double max_time;
        int max_nodes;

        /** plan properties **/
        int start_idx;
        int goal_idx; // start and goal index
        int start_x, start_y;
        int goal_x, goal_y;

    public:
        ThetastarPlanner(/* args */);
        ThetastarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        ThetastarPlanner(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      double tolerance, std::vector<geometry_msgs::PoseStamped> &plan);

        /** publishers and services**/
        void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
        // void publishPotential(float * cost_array);
        void publishPotential(std::vector<double> cost_array);

        bool makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

        void saveCostmapProperties(costmap_2d::Costmap2D *costmap);

        int mapToIndex(const int &mx, const int &my);

        void clearRobotCell(unsigned int mx, unsigned int my);

        void setCostmap(costmap_2d::Costmap2DROS *costmap_ros);

        float calculatehcost(const int &current_idx);

        void indexToMap(const int &idx, int &mx, int &my);

        geometry_msgs::PoseStamped indexToPose(const int idx, ros::Time plan_time);

        void mapToWorld(double &wx, double &wy, const int &mx, const int &my);
        
        void reset();

        std::vector<int> getNeighbors(const int &current_idx);

        void addToOpenList(int &current_idx, int &nb_idx);

        bool isFree(int &idx);

        int costState(const int &idx);

        void updateVertex(int &current_idx, int &nb_idx);

        bool lineOfSight(int &idx1, int &idx2);

        bool isCheap(int mx, int my);

        float calculatedist(const int &current_idx, const int &nb_idx);

        // void publishPotential(std::vector<double> cost_array);
        void MaxNodes();

        bool extractGradientPath(std::vector<double> potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float>> &path);

        float gradCell(std::vector<double> potential, int n);

        void MaxTime(double ttrack);

        ros::Publisher potential_pub_;
        ros::Publisher plan_pub_;
    };

} // namespace thetastar_planner

#endif