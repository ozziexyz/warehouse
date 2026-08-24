#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <warehouse_interfaces/srv/generate_path.hpp>

using GraphNode = std::pair<int, int>;
using GeneratePath = warehouse_interfaces::srv::GeneratePath;

class AStarGrid {
    public:
        AStarGrid(int width, int height) : width(width), height(height) {}

        std::vector<GraphNode> generate_path(GraphNode start, GraphNode end) {
            std::set<GraphNode> blocked;
            for (int col = 0; col <= 4; ++col) {
                for (int row : {1, 3, 5, 7}) {
                    blocked.insert({col, row});
                }
            }

            return astar(start, end, blocked);
        }

        bool in_bounds(const GraphNode &node) const {
            return node.first >= 0 && node.first < width && node.second >= 0 && node.second < height;
        }

    private:
        std::vector<GraphNode> neighbors(const GraphNode &node) {
            static const int dx[] = {1, -1, 0, 0};
            static const int dy[] = {0, 0, 1, -1};
            std::vector<GraphNode> result;
            for (int i = 0; i < 4; ++i) {
                int nx = node.first + dx[i];
                int ny = node.second + dy[i];
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    result.emplace_back(nx, ny);
                }
            }
            return result;
        }

        int heuristic(const GraphNode &a, const GraphNode &b) {
            return std::abs(a.first - b.first) + std::abs(a.second - b.second);
        }

        std::vector<GraphNode> astar(const GraphNode &start, const GraphNode &goal, const std::set<GraphNode> &blocked) {
            using QueueEntry = std::pair<int, GraphNode>;
            std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open_heap;
            open_heap.push({heuristic(start, goal), start});

            std::map<GraphNode, GraphNode> came_from;
            std::map<GraphNode, int> g_score;
            g_score[start] = 0;
            std::set<GraphNode> visited;

            while (!open_heap.empty()) {
                GraphNode current = open_heap.top().second;
                open_heap.pop();

                if (current == goal) {
                    std::vector<GraphNode> path{current};
                    while (came_from.count(current)) {
                        current = came_from[current];
                        path.push_back(current);
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }

                if (visited.count(current)) {
                    continue;
                }
                visited.insert(current);

                for (const GraphNode &neighbor : neighbors(current)) {
                    if (blocked.count(neighbor)) {
                        continue;
                    }
                    int tentative_g = g_score[current] + 1;
                    int existing = g_score.count(neighbor) ? g_score[neighbor] : std::numeric_limits<int>::max();
                    if (tentative_g < existing) {
                        came_from[neighbor] = current;
                        g_score[neighbor] = tentative_g;
                        int f_score = tentative_g + heuristic(neighbor, goal);
                        open_heap.push({f_score, neighbor});
                    }
                }
            }

            return {};
        }

        int width;
        int height;
};

class PathPlanner : public rclcpp::Node {
    public:
        PathPlanner() : Node("path_planner") {
            RCLCPP_INFO(get_logger(), "path_planner node started");
            
            declare_parameter<int>("grid_width", 7);
            declare_parameter<int>("grid_height", 9);
            declare_parameter<double>("resolution", 1.0);
            declare_parameter<double>("origin_x", -2.0);
            declare_parameter<double>("origin_y", 4.0);
            declare_parameter<double>("waypoint_density", 20.0);

            grid_width = get_parameter("grid_width").as_int();
            grid_height = get_parameter("grid_height").as_int();
            resolution = get_parameter("resolution").as_double();
            origin_x = get_parameter("origin_x").as_double();
            origin_y = get_parameter("origin_y").as_double();
            waypoint_density = get_parameter("waypoint_density").as_double();
            

            grid = std::make_unique<AStarGrid>(grid_width, grid_height);

            generate_path_service = create_service<GeneratePath>(
                "/generate_path",
                std::bind(&PathPlanner::generate_path_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
        GraphNode world_to_grid(const geometry_msgs::msg::Point &point) {
            return {
                static_cast<int>(std::lround((point.x - origin_x) / resolution)),
                static_cast<int>(std::lround((origin_y - point.y) / resolution))
            };
        }

        geometry_msgs::msg::PoseStamped grid_to_world(const GraphNode &node, const std_msgs::msg::Header &header) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.pose.position.x = origin_x + node.first * resolution;
            pose.pose.position.y = origin_y - node.second * resolution;
            pose.pose.orientation.w = 1.0;
            return pose;
        }

        void generate_path_callback(const GeneratePath::Request::SharedPtr req, GeneratePath::Response::SharedPtr resp) {
            GraphNode start = world_to_grid(req->start.pose.position);
            GraphNode goal = world_to_grid(req->goal.pose.position);

            if (!grid->in_bounds(start) || !grid->in_bounds(goal)) {
                RCLCPP_WARN(get_logger(), "Start or goal is outside the planning grid");
                resp->success = false;
                return;
            }

            std::vector<GraphNode> grid_path = grid->generate_path(start, goal);
            if (grid_path.empty()) {
                RCLCPP_WARN(get_logger(), "No path found between requested start and goal");
                resp->success = false;
                return;
            }

            nav_msgs::msg::Path path;
            path.header.frame_id = req->start.header.frame_id;
            path.header.stamp = get_clock()->now();
            for (auto it = grid_path.begin(); it != grid_path.end(); ++it) {
                geometry_msgs::msg::PoseStamped waypoint = grid_to_world(*it, path.header);
                path.poses.push_back(grid_to_world(*it, path.header));
                if(it != grid_path.end() - 1) {
                    geometry_msgs::msg::PoseStamped next_waypoint = grid_to_world(*std::next(it, 1), path.header);
                    double dx = next_waypoint.pose.position.x - waypoint.pose.position.x;
                    double dy = next_waypoint.pose.position.y - waypoint.pose.position.y;
                    double d = hypot(dx, dy);
                    RCLCPP_INFO(get_logger(), "dx: %f, dy: %f, d: %f", dx, dy, d);
                    int num_points = floor(d * waypoint_density);
                    for(int i = 0; i < num_points - 1; i++) {
                        geometry_msgs::msg::PoseStamped filler_waypoint;
                        filler_waypoint.header.frame_id = "odom";
                        filler_waypoint.pose.position.x = (i + 1) * dx / (num_points) + waypoint.pose.position.x;
                        filler_waypoint.pose.position.y = (i + 1) * dy / (num_points) + waypoint.pose.position.y;
                        path.poses.push_back(filler_waypoint);
                    };
                }
            }

            resp->path = path;
            resp->success = true;
        }

        rclcpp::Service<GeneratePath>::SharedPtr generate_path_service;
        std::unique_ptr<AStarGrid> grid;
        int grid_width;
        int grid_height;
        double resolution;
        double origin_x;
        double origin_y;
        double waypoint_density;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
