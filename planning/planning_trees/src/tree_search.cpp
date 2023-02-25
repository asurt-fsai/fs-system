#include "../include/tree_search.h"

TreeSearch::TreeSearch(std::vector<Cone> cones, TreeSearchParams params){
    this->params = params;
    this->cones = TreeSearch::filterLocal(cones, this->params.field_of_view, this->params.distance);
    this->waypoints = TreeSearch::triangulate(this->cones);
}

template <typename T> std::vector<T> TreeSearch::filterLocal(std::vector<T> points, float field_of_view, float distance){
    return TreeSearch::filterLocal(points, field_of_view, distance, 0, 0, 0);
}

std::vector<Cone> TreeSearch::filterLocal(std::vector<Cone> points, float field_of_view, float distance, float x, float y, float heading){
    std::vector<Cone> filtered_points;

    for(Cone point: points){
        float dx = point.x - x;
        float dy = point.y - y;
        float dist = sqrt(dx*dx + dy*dy);
        float angle = atan2(dy, dx) - heading;

        if(angle < -M_PI) angle += 2 * M_PI; // Normalize angle to [-pi, pi]
        if(angle > M_PI) angle -= 2 * M_PI; // Normalize angle to [-pi, pi]

        if (fabs(angle) <= field_of_view/2 && dist <= distance){
            filtered_points.push_back(point);
        }
    }
    return filtered_points;
}

std::vector<Waypoint*> TreeSearch::filterLocal(std::vector<Waypoint*> points, float field_of_view, float distance, float x, float y, float heading){
    std::vector<Waypoint*> filtered_points;

    for (int i = 0; i < points.size(); i++){
        float dx = points[i]->x - x;
        float dy = points[i]->y - y;
        float dist = sqrt(dx*dx + dy*dy);
        float angle = atan2(dy, dx) - heading;

        if(angle < -M_PI) angle += 2 * M_PI; // Normalize angle to [-pi, pi]
        if(angle > M_PI) angle -= 2 * M_PI; // Normalize angle to [-pi, pi]

        if (fabs(angle) <= field_of_view/2 && dist <= distance){
            filtered_points.push_back(points[i]);
        }
    }
    return filtered_points;
}

std::vector<Waypoint> TreeSearch::triangulate(std::vector<Cone> cones){
    std::vector<Waypoint> waypoints;
    for(Cone cone: cones){
        // Add 8 waypoints around the cone
        for(int i = 0; i < 8; i++){
            float angle = i * M_PI / 4;
            float x = cone.x + this->params.triangulation_radius * cos(angle);
            float y = cone.y + this->params.triangulation_radius * sin(angle);

            // find the closest cone to the waypoint
            Waypoint w(x, y);
            float closest_cone_dist = w.getDistNearestCone(cones);

            // Find closest waypoint to this waypoint
            float closest_waypoint_dist = std::numeric_limits<float>::infinity();
            for (int i = 0; i < waypoints.size(); i++){
                float dist = w.getDistanceTo(waypoints[i]);
                if (dist < closest_waypoint_dist){
                    closest_waypoint_dist = dist;
                }
            }
            if(closest_cone_dist > this->params.triangulation_min_cone_dist && closest_waypoint_dist > this->params.triangulation_min_waypoint_dist){
                waypoints.emplace_back(x, y);
            }
        }
    }
    return waypoints;
}

Path TreeSearch::getPath(){
    std::queue<Path*> paths;
    std::queue<float> costs;
    Path start_path = Path();
    paths.push(&start_path);
    costs.push(std::numeric_limits<float>::infinity());
    std::vector<Waypoint *> pointers;
    for (int i = 0; i < this->waypoints.size();i++){
        pointers.push_back(&this->waypoints[i]);
    }

    for (int i = 0; i < params.max_search_iterations; i++)
    {
        bool no_new_paths = true;
        int size = paths.size();
        for (int j = 0; j < size; j++)
        {
            Path *curr_path = paths.front();
            float curr_cost = costs.front();
            paths.pop();
            costs.pop();

            if (curr_path->waypoints.size() > params.max_waypoints_per_path)
            {
                paths.push(curr_path);
                costs.push(curr_cost);
                continue;
            }
            Waypoint *last_waypoint = &curr_path->waypoints.back();

            std::vector<Waypoint *> possible_next_waypoints = filterLocal(pointers, params.waypoint_field_of_view, params.waypoint_distance, last_waypoint->x, last_waypoint->y, last_waypoint->heading);
            bool added_point = false;
            for (int k = 0; k < possible_next_waypoints.size(); k++)
            {
                Path new_path = Path();  // TODO: copy constructor
                new_path.addWaypoint(this->waypoints[k].x, this->waypoints[k].y);
                float new_cost = new_path.getCost();

                if (paths.size() > this->params.path_queue_limit){

                }
            }
        }
    }

    // Example on using a queue in cpp
    // std::queue<int> q;
    // q.push(1);
    // q.push(2);
    // q.push(3);
    // q.push(4);
    // q.push(5);
    // while(!q.empty()){
    //     std::cout << q.front() << std::endl;
    //     q.pop();
    // }

}
