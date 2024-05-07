#include <thetastar_planner/thetastar_planner.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(thetastar_planner::ThetastarPlanner, nav_core::BaseGlobalPlanner)

namespace thetastar_planner
{
    ThetastarPlanner::ThetastarPlanner() : costmap_(NULL), initialized_(false), allow_unknown_(true) {}

    ThetastarPlanner::ThetastarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : costmap_(NULL), initialized_(false), allow_unknown_(true)
    {
        initialize(name, costmap_ros);
    }

    ThetastarPlanner::ThetastarPlanner(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame) : costmap_(NULL), initialized_(false), allow_unknown_(true)
    {
        initialize(name, costmap, global_frame);
    }

    void ThetastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        costmap_ros_ = costmap_ros;
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void ThetastarPlanner::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);

            global_frame_ = global_frame;
            costmap_ = costmap;
            saveCostmapProperties(costmap_);

            gradx_ = new float[ns_];
            grady_ = new float[ns_];
            pathStep_ = 0.5;

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            private_nh.param("allow_unknown", allow_unknown_, false);
            private_nh.param("neutral_cost", neutral_cost_, 50);
            private_nh.param("cost_factor", cost_factor_, 0.8);
            private_nh.param("obstacle_cost_ratio", obs_cost_ratio, 1.0);
            private_nh.param("cheap_threshold", cheapthreshold, 150);
            private_nh.param("get_grad_path", get_grad_path_, false);

            private_nh.param("default_tolerance", default_tolerance_, 0.0);
            private_nh.param("publish_potential", publish_potential_, true);
            private_nh.param("publish_scale", publish_scale_, 100);

            bool allow_obs_plan_through = false;

            counter = 0;
            max_time = 0;
            max_nodes = 0;

            initialized_ = true;
        }
        else
        {
            ROS_WARN("This planner is already initialized... passing");
        }
    }

    void ThetastarPlanner::saveCostmapProperties(costmap_2d::Costmap2D *costmap)
    {
        nx_ = costmap_->getSizeInCellsX();
        ny_ = costmap_->getSizeInCellsY();
        ox_ = costmap_->getOriginX();
        oy_ = costmap_->getOriginY();
        res_ = costmap_->getResolution();
        ns_ = nx_ * ny_;
        cmlocal.resize(ns_);
    }

    bool ThetastarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                    const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        return makePlan(start, goal, default_tolerance_, plan);
    }

    bool ThetastarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        ROS_INFO("KONTOL");
        if (!initialized_)
        {
            ROS_ERROR("Initialize before calling makePlan");
            return false;
        }

        plan.clear();

        reset();

        g_costs.resize(ns_);
        parents.resize(ns_);
        f_costs = new float[ns_];

        std::fill(f_costs, f_costs + ns_, MAX_COST);
        std::fill(g_costs.begin(), g_costs.end(), MAX_COST); // initialize every g-cost with highest possible cost

        // Check frames are equal: until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
        if (goal.header.frame_id != global_frame_)
        {
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
            return false;
        }
        if (start.header.frame_id != global_frame_)
        {
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), start.header.frame_id.c_str());
            return false;
        }

        // Check start and goal positions are within bounds
        double start_x_w = start.pose.position.x;
        double start_y_w = start.pose.position.y;
        double goal_x_w = goal.pose.position.x;
        double goal_y_w = goal.pose.position.y;
        unsigned int start_x_, start_y_, goal_x_, goal_y_;

        if (!costmap_->worldToMap(start_x_w, start_y_w, start_x_, start_y_))
        {
            ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
            return false;
        }
        if (!costmap_->worldToMap(goal_x_w, goal_y_w, goal_x_, goal_y_))
        {
            ROS_WARN_THROTTLE(1.0, "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }

        start_x = start_x_;
        start_y = start_y_;
        goal_x = goal_x_;
        goal_y = goal_y_;

        start_idx = mapToIndex(start_x, start_y);
        goal_idx = mapToIndex(goal_x, goal_y);

        clearRobotCell(start_x, start_y);

        setCostmap(costmap_ros_);

        g_costs[start_idx] = 0; // set gcost of start cell as 0
        openList.push_back(Cell(start_idx, 0));
        std::fill(parents.begin(), parents.end(), -1);

        float f_cost = g_costs[start_idx] + calculatehcost(start_idx);
        f_costs[start_idx] = f_cost;

        int cycles = 0;
        int max_cycles = nx_ * ny_ * 2;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        while (!openList.empty() && cycles++ < max_cycles)
        {
            std::pop_heap(openList.begin(), openList.end(), greater());
            Cell current_cell = openList.back();
            openList.pop_back();

            int current_idx = current_cell.idx;

            if (current_idx == goal_idx)
            {
                ros::Time plan_time = ros::Time::now();
                geometry_msgs::PoseStamped pose;
                std::vector<geometry_msgs::PoseStamped> plan_reversed; // reversed plan, we need to reverse this for move_base

                while (parents[current_idx] != -1) // since start cell's parents = -1, it will stop at start_cell
                {
                    pose = indexToPose(current_idx, plan_time); // convert index to world coordinates
                    plan_reversed.push_back(pose);
                    current_idx = parents[current_idx]; // current_idx is pointing to parent cell
                }

                // add start cell to plan
                pose = indexToPose(current_idx, plan_time);
                plan_reversed.push_back(pose);

                // reverse the plan
                int len_plan = plan_reversed.size();
                for (int i = len_plan - 1; i >= 0; --i)
                {
                    plan.push_back(plan_reversed[i]);
                }
                ROS_INFO("KONTOL ASU");
                publishPlan(plan);
                ROS_INFO("KONTOL ASU4");
                break;
            }

            ROS_INFO("KONTOL ASU3");
            visited.insert(current_idx);

            std::vector<int> neighbor_indices = getNeighbors(current_idx); // Get free neighbors
            for (int nb_idx : neighbor_indices)
            {
                addToOpenList(current_idx, nb_idx);
            }
        }

        if (get_grad_path_)
        {
            ROS_INFO("Extracting gradient path");
            plan.clear();

            std::vector<std::pair<float, float>> path;
            if (extractGradientPath(g_costs, start_x, start_y, goal_x, goal_y, path))
            {
                ROS_INFO("Gradient path succeeded!");

                ros::Time plan_time = ros::Time::now();
                for (int i = path.size() - 1; i >= 0; i--)
                {
                    std::pair<float, float> point = path[i];
                    // convert the plan to world coordinates
                    double world_x, world_y;
                    mapToWorld(world_x, world_y, point.first, point.second);

                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = plan_time;
                    pose.header.frame_id = global_frame_;
                    pose.pose.position.x = world_x;
                    pose.pose.position.y = world_y;
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    plan.push_back(pose);
                }
            }
            else
            {
                ROS_ERROR("Path not found by gradient descent");
            }
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        ROS_INFO("Time taken: %.5f , No. of loops: %d", ttrack, cycles);

        MaxTime(ttrack);

        ROS_INFO("Index: %2.3f", f_cost);
        ROS_INFO("ASU");

        delete f_costs;

        return !plan.empty();
    }

    void ThetastarPlanner::MaxTime(double ttrack)
    {
        if (ttrack > max_time)
        {
            max_time = ttrack;
        }
        counter += 1;
        ROS_INFO("Max time taken: %f", max_time);
    }

    void ThetastarPlanner::MaxNodes()
    {
        int nodes = 0;
        for (int i = 0; i < ns_; i++)
        {
            if (g_costs[i] < 1e10)
                nodes++;
        }
        if (max_nodes < nodes)
            max_nodes = nodes;
        std::cout << "Maximum nodes expanded: " << max_nodes << std::endl;
    }

    bool ThetastarPlanner::extractGradientPath(std::vector<double> potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float>> &path)
    {
        std::pair<float, float> current;
        int stc = mapToIndex(goal_x, goal_y);

        // set up offset
        float dx = goal_x - (int)goal_x;
        float dy = goal_y - (int)goal_y;
        int ns = nx_ * ny_;
        memset(gradx_, 0, ns * sizeof(float));
        memset(grady_, 0, ns * sizeof(float));

        int c = 0;
        while (c++ < ns * 4)
        {
            double nx = stc % nx_ + dx, ny = stc / nx_ + dy; // get map coordinates of cell index stc

            // check if the cell index stc is near the start cell
            if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5)
            {
                current.first = start_x;
                current.second = start_y;
                path.push_back(current);
                return true;
            }

            // check if cell index stc is out of bounds
            if (stc < nx_ || stc > nx_ * ny_ - nx_)
            {
                printf("[PathCalc] Out of bounds\n");
                return false;
            }

            current.first = nx;
            current.second = ny;

            // ROS_INFO("%d %d | %f %f ", stc%nx_, stc/nx_, dx, dy);

            path.push_back(current); // push back goal cell first

            // check for any oscillation in path
            bool oscillation_detected = false;
            int npath = path.size();
            if (npath > 2 && path[npath - 1].first == path[npath - 3].first && path[npath - 1].second == path[npath - 3].second)
            {
                ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
                oscillation_detected = true;
            }

            int stcnx = stc + nx_; // set up values for checking in octile directions
            int stcpx = stc - nx_;

            // If any neighboring cell is obstacle or over inflated or oscillation is true,
            // then check for potentials at eight positions near cell
            if (potential[stc] >= MAX_COST || potential[stc + 1] >= MAX_COST || potential[stc - 1] >= MAX_COST || potential[stcnx] >= MAX_COST || potential[stcnx + 1] >= MAX_COST || potential[stcnx - 1] >= MAX_COST || potential[stcpx] >= MAX_COST || potential[stcpx + 1] >= MAX_COST || potential[stcpx - 1] >= MAX_COST || oscillation_detected)
            {
                ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int)path.size());
                // check eight neighbors to find the lowest
                int minc = stc;            // index of current cell
                int minp = potential[stc]; // potential of current cell
                // check each  neighbor cell in a counter clockwise fashion starting from the bottom cell
                int st = stcpx - 1;
                if (potential[st] < minp)
                {
                    minp = potential[st]; //
                    minc = st;
                }
                st++;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                st++;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                st = stc - 1;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                st = stc + 1;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                st = stcnx - 1;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                st++;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                st++;
                if (potential[st] < minp)
                {
                    minp = potential[st];
                    minc = st;
                }
                stc = minc;
                dx = 0;
                dy = 0;

                // ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
                //     potential[stc], path[npath-1].first, path[npath-1].second);

                if (potential[stc] >= MAX_COST)
                { // surrounded by obstacles
                    ROS_DEBUG("[PathCalc] No path found, high potential");
                    // savemap("navfn_highpot");
                    return 0;
                }
            }

            //
            // else current cell neighbors are free (not an obstacle or over inflated)
            else
            {

                // get grad at current cell, top cell, top-right, and right.
                gradCell(potential, stc);
                gradCell(potential, stc + 1);
                gradCell(potential, stcnx);
                gradCell(potential, stcnx + 1);

                // get interpolated gradient
                float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
                float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
                float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
                float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
                float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
                float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

                // show gradients
                ROS_DEBUG(
                    "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc + 1], grady_[stc + 1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx + 1], grady_[stcnx + 1], x, y);

                // check for zero gradient, failed
                if (x == 0.0 && y == 0.0)
                {
                    ROS_DEBUG("[PathCalc] Zero gradient");
                    return 0;
                }

                // move in the right direction
                float ss = pathStep_ / hypot(x, y);
                dx += x * ss;
                dy += y * ss;

                // check for overflow
                // if overflow, then move the index in that direction
                if (dx > 1.0)
                {
                    stc++;
                    dx -= 1.0;
                }
                if (dx < -1.0)
                {
                    stc--;
                    dx += 1.0;
                }
                if (dy > 1.0)
                {
                    stc += nx_;
                    dy -= 1.0;
                }
                if (dy < -1.0)
                {
                    stc -= nx_;
                    dy += 1.0;
                }
            }

            printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
                   potential[stc], dx, dy, path[npath - 1].first, path[npath - 1].second);
        }

        return false;
    }

    float ThetastarPlanner::gradCell(std::vector<double> potential, int n)
    {
        if (gradx_[n] + grady_[n] > 0.0) // check if value is already assigned
            return 1.0;

        if (n < nx_ || n > nx_ * ny_ - nx_) // check if index is out of bounds
            return 0.0;
        float cv = potential[n];
        float dx = 0.0;
        float dy = 0.0;

        // if current cell is "in an obstacle"
        // then assign a lethal cost to dx and dy
        if (cv >= MAX_COST)
        {
            if (potential[n - 1] < MAX_COST) // left and right
                dx = -costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            else if (potential[n + 1] < MAX_COST)
                dx = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

            if (potential[n - nx_] < MAX_COST) // up and down
                dy = -costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            else if (potential[n + nx_] < MAX_COST)
                dy = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        }

        else // not in an obstacle
        {
            // assign to dx and dy the difference in cost between current cell and neighboring cell
            if (potential[n - 1] < MAX_COST)
                dx += potential[n - 1] - cv;
            if (potential[n + 1] < MAX_COST)
                dx += cv - potential[n + 1];

            if (potential[n - nx_] < MAX_COST)
                dy += potential[n - nx_] - cv;
            if (potential[n + nx_] < MAX_COST)
                dy += cv - potential[n + nx_];
        }

        // normalize
        // then assign gradient value to the gradient array
        float norm = hypot(dx, dy); // returns hypothenuse
        if (norm > 0)
        {
            norm = 1.0 / norm;
            gradx_[n] = norm * dx;
            grady_[n] = norm * dy;
        }
        return norm;
    }

    int ThetastarPlanner::costState(const int &idx)
    {
        if (idx <= 0 || idx >= ns_)
            return costmap_2d::LETHAL_OBSTACLE;
        return cmlocal[idx];
    }

    bool ThetastarPlanner::isFree(int &idx)
    {
        int cost = costState(idx);
        int obs_cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE * obs_cost_ratio;
        obs_cost = obs_cost * cost_factor_ + neutral_cost_;
        bool check = allow_obs_plan_through ? (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) : (cost < obs_cost); // check if cost is free (< costmap_2d::INSCRIBED_INFLATED_OBSTACLE) or unknown (< obs_cost)
        check = allow_unknown_ ? (check || cost == costmap_2d::NO_INFORMATION) : check;

        return check;
    }

    void ThetastarPlanner::addToOpenList(int &current_idx, int &nb_idx)
    {
        if (!isFree(nb_idx))
            return;

        if (visited.count(nb_idx))
            return;

        updateVertex(current_idx, nb_idx);
    }

    void ThetastarPlanner::updateVertex(int &current_idx, int &nb_idx)
    {
        float old_cost = g_costs[nb_idx];
        float tg_cost;

        if (lineOfSight(parents[current_idx], nb_idx))
        {
            tg_cost = g_costs[parents[current_idx]] + calculatedist(parents[current_idx], nb_idx);
            if (tg_cost < g_costs[nb_idx])
            {
                parents[nb_idx] = parents[current_idx];
                g_costs[nb_idx] = tg_cost;
            }
        }
        else
        {
            tg_cost = g_costs[current_idx] + calculatedist(current_idx, nb_idx);
            if (tg_cost < g_costs[nb_idx])
            {
                parents[nb_idx] = current_idx;
                g_costs[nb_idx] = tg_cost;
            }
        }

        if (g_costs[nb_idx] < old_cost)
        {

            f_costs[nb_idx] = g_costs[nb_idx] + calculatehcost(nb_idx);
            openList.push_back(Cell(nb_idx, f_costs[nb_idx]));
            std::push_heap(openList.begin(), openList.end(), greater()); // sorts last element into the minheap
            // std::push_heap(openList.begin(), openList.end(), lesser()); //sorts last element into the maxheap
        }
    }

    bool ThetastarPlanner::lineOfSight(int &idx1, int &idx2)
    {
        int sx1, sy1, sx2, sy2;
        int x0, x1, y1, y0;
        indexToMap(idx1, sx1, sy1);
        indexToMap(idx2, sx2, sy2);
        x0 = sx1;
        y0 = sy1;
        x1 = sx2;
        y1 = sy2;

        int dx = x1 - x0;
        int dy = y1 - y0;
        int f = 0;

        if (dy < 0)
        {
            dy = -dy;
            sy1 = -1;
        }
        else
            sy1 = 1;

        if (dx < 0)
        {
            dx = -dx;
            sx1 = -1;
        }
        else
            sx1 = 1;

        int sx1_ = (sx1 - 1) / 2;
        int sy1_ = (sy1 - 1) / 2;

        if (dx >= dy)
        {
            while (x0 != x1)
            {
                f += dy;
                if (f >= dx)
                {
                    if (!isCheap(x0 + sx1_, y0 + sy1_)) // isCheap(x,y, thres) is TRUE if the cell is within the map and below the cost threshold
                        return false;
                    y0 += sy1;
                    f -= dx;
                }
                if (f != 0 && !isCheap(x0 + sx1_, y0 + sy1_))
                    return false;
                if (dy == 0 && !isCheap(x0 + sx1_, y0) && !isCheap(x0 + sx1_, y0 - 1))
                    return false;
                x0 += sx1;
            }
        }
        else
        {
            while (y0 != y1)
            {
                f += dx;
                if (f >= dy)
                {
                    if (!isCheap(x0 + sx1_, y0 + sy1_))
                        return false;
                    x0 += sx1;
                    f -= dy;
                }
                if (f != 0 && !isCheap(x0 + sx1_, y0 + sy1_))
                    return false;
                if (dx == 0 && !isCheap(x0, y0 + sy1_) && !isCheap(x0 - 1, y0 + sy1_))
                    return false;
                y0 += sy1;
            }
        }
        return true;
    }

    bool ThetastarPlanner::isCheap(int mx, int my)
    {
        int idx = mx + my * nx_;
        if (idx <= 0 || idx >= ns_)
            return false;
        float cost = cmlocal[idx];
        return (cost < cheapthreshold);
    }

    std::vector<int> ThetastarPlanner::getNeighbors(const int &current_idx)
    {
        std::vector<int> neighbors{nx_, nx_ + 1, 1, -nx_ + 1, -nx_, -nx_ - 1, -1, nx_ - 1}; // in octile direction (From top cell in clockwise fashion)
        std::vector<int> neighbor_indices;
        // add in octile direction (From top cell in clockwise fashion)
        for (int nb : neighbors)
        {
            int nb_to_be_added = current_idx + nb;
            neighbor_indices.push_back(nb_to_be_added);
        }
        return neighbor_indices;
    }

    void ThetastarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        ROS_INFO("KONTOL ASU2");
        if (!initialized_)
        {
            ROS_INFO("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        if (path.empty()) // check for empty path
        {
            gui_path.header.frame_id = global_frame_; // still set a valid frame so visualization won't hit transform issues
            gui_path.header.stamp = ros::Time::now();
        }
        else
        {
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++)
        {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    geometry_msgs::PoseStamped ThetastarPlanner::indexToPose(const int idx, ros::Time plan_time)
    {
        int mx;
        int my; // map coordinates
        double wx;
        double wy; // world coordinates
        indexToMap(idx, mx, my);
        mapToWorld(wx, wy, mx, my);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        return pose;
    }

    void ThetastarPlanner::mapToWorld(double &wx, double &wy, const int &mx, const int &my)
    {
        wx = (mx * res_);
        wx = ox_ + wx;
        wy = (my * res_);
        wy = oy_ + wy;
    }

    void ThetastarPlanner::reset()
    {
        openList.clear();
        visited.clear();
        g_costs.clear();
        parents.clear();
    }

    int ThetastarPlanner::mapToIndex(const int &mx, const int &my)
    {
        return mx + (my * nx_);
    }

    void ThetastarPlanner::indexToMap(const int &idx, int &mx, int &my)
    {
        my = idx / nx_; // floor
        // mx = (my * nx_);
        mx = idx - my * nx_;
    }

    void ThetastarPlanner::clearRobotCell(unsigned int mx, unsigned int my)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        costmap_->setCost(mx, my, costmap_2d::FREE_SPACE); // set the associated costs in the cost map to be free
    }

    void ThetastarPlanner::setCostmap(costmap_2d::Costmap2DROS *costmap_ros)
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros->getCostmap()->getMutex()));
        COSTTYPE *cmap = costmap_ros->getCostmap()->getCharMap();

        for (int i = 0; i < ns_; i++, cmap++)
        {
            cmlocal[i] = costmap_2d::LETHAL_OBSTACLE;
            int v = *cmap;
            if (v < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) // if not obstacle
            {
                v = neutral_cost_ + cost_factor_ * v;    // assign neutral cost to current cell
                if (v >= costmap_2d::LETHAL_OBSTACLE)    // if > forbidden region (254)
                    v = costmap_2d::LETHAL_OBSTACLE - 1; // make it an obstacle (253) or > forbidden
                cmlocal[i] = v;                          // assign to local cost array
            }
            else if (v == costmap_2d::NO_INFORMATION && allow_unknown_) // if unknown and unknowns are allowed
            {
                v = costmap_2d::NO_INFORMATION; // assign unknown cost (255)
                cmlocal[i] = v;
            }
        }
    }

    float ThetastarPlanner::calculatehcost(const int &current_idx)
    {
        int mx;
        int my;

        indexToMap(current_idx, mx, my);

        int X = abs(goal_x - mx);
        int Y = abs(goal_y - my);

        return std::sqrt(X * X + Y * Y) * neutral_cost_;
    }

    float ThetastarPlanner::calculatedist(const int &current_idx, const int &nb_idx)
    {
        int mx1;
        int my1;
        int mx2;
        int my2;

        indexToMap(current_idx, mx1, my1);
        indexToMap(nb_idx, mx2, my2);

        int X = abs(mx2 - mx1);
        int Y = abs(my2 - my1);

        return std::sqrt(X * X + Y * Y) * costState(nb_idx);
    }

    void ThetastarPlanner::publishPotential(std::vector<double> cost_array)
    {
        nav_msgs::OccupancyGrid grid;

        // Publish Whole Grid
        grid.header.frame_id = global_frame_;
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = res_;

        grid.info.width = nx_;
        grid.info.height = ny_;

        double wx, wy;
        costmap_->mapToWorld(0, 0, wx, wy);
        grid.info.origin.position.x = wx - res_ / 2;
        grid.info.origin.position.y = wy - res_ / 2;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(nx_ * ny_);

        float max = 0.0;
        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            float potential = cost_array[i];
            if (potential < MAX_COST)
            {
                if (potential > max)
                {
                    max = potential;
                }
            }
        }

        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            if (cost_array[i] >= MAX_COST)
            {
                grid.data[i] = -1;
            }
            else
                grid.data[i] = cost_array[i] * publish_scale_ / max;
        }

        potential_pub_.publish(grid);
    }

}