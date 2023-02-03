#include "../include/tree_search.h"

TreeSearch::TreeSearch(std::vector<Cone> cones, TreeSearchParams params){
    this->params = params;
    this->cones = TreeSearch::filterLocal(cones, this->params.field_of_view, this->params.distance);
    this->waypoints = TreeSearch::triangulate(this->cones);
}

Path TreeSearch::getPath(){
    if(cones.size()==0 || waypoints.size()==0){  // TODO: fix this
        return Path();
    }

    std::vector<std::tuple<Path, float>> paths; // Contains list of current path exploring and their costs

    for (int i = 0; i < params.max_search_iterations; i++){
        bool has_new_paths = false;
    }
}

std::vector<Cone> TreeSearch::filterLocal(std::vector<Cone> cones, float field_of_view, float distance){
    return TreeSearch::filterLocal(cones, field_of_view, distance, 0, 0, 0);
}

std::vector<Cone> TreeSearch::filterLocal(std::vector<Cone> cones, float field_of_view, float distance, float x, float y, float heading){
    std::vector<Cone> filtered_cones;

    for(Cone cone: cones){
        float dx = cone.x - x;
        float dy = cone.y - y;
        float dist = sqrt(dx*dx + dy*dy);
        float angle = atan2(dy, dx) - heading;

        if(angle < -M_PI) angle += 2 * M_PI; // Normalize angle to [-pi, pi]
        if(angle > M_PI) angle -= 2 * M_PI; // Normalize angle to [-pi, pi]

        if (fabs(angle) <= field_of_view/2 && dist <= distance){
            filtered_cones.push_back(cone);
        }
    }
    return filtered_cones;
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
