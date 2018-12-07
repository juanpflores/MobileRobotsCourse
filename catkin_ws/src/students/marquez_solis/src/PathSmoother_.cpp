#include "PathSmoother.h"
#include "futils.h"
using std::vector;
constexpr int smooth_max_iter = 30;
nav_msgs::OccupancyGrid PathSmoother::InflateObstacles(nav_msgs::OccupancyGrid& map,  float inflation_radius)
{
    nav_msgs::OccupancyGrid inflated = map;
    int inflation_cells = (int)(inflation_radius / map.info.resolution);
    int kernel_size     = (2*inflation_cells + 1) * (2*inflation_cells + 1);
    std::vector<int> neighbors;
    neighbors.resize(kernel_size);

    for(int i=-inflation_cells, counter = 0; i<=inflation_cells; i++)
        for(int j=-inflation_cells; j<=inflation_cells; j++, counter++)
            neighbors[counter] = j*map.info.width + i;

    for(size_t i=0; i < map.data.size(); i++)
	if(map.data[i] > 40)
	    for(size_t j=0; j < neighbors.size(); j++)
		inflated.data[i + neighbors[j]] = 100;
    
    return inflated;
}

inline int posGrid(int x,int y,int w){
    return x+y*w;
}

nav_msgs::OccupancyGrid PathSmoother::GetNearnessMap(nav_msgs::OccupancyGrid& map, float nearness_radius)
{
    if(nearness_radius <= 0)
	return map;	
    nav_msgs::OccupancyGrid nearnessMap = map;
    auto &data = map.data;
    auto &out = nearnessMap.data;
    auto w = map.info.width;
    int max_rad= std::round(nearness_radius/map.info.resolution);
    for(auto &e : out)
        e = 0;
    for(int i = 0; i < data.size();i++){
        if(data[i] < 40) continue;
        for(int rad = 1;rad <= max_rad;rad++){
            int near_value = max_rad -rad + 1;
            
            for(int k = -rad;k <= rad;k++){
                int idx = i + posGrid(k,-rad,w);
                if (data[idx] > 60) out[idx] = 100;
                if(out[idx] < near_value)
                    out[idx] = near_value;
            }
            for(int k = -rad;k <= rad;k++){
                int idx = i + posGrid(k,rad,w);
                if (data[idx] > 60) out[idx] = 100;
                else if(out[idx] < near_value)
                    out[idx] = near_value;
            }
            for(int k = -rad + 1;k < rad;k++){
                int idx = i + posGrid(rad,k,w);
                if (data[idx] > 60) out[idx] = 100;
                else if(out[idx] < near_value)
                    out[idx] = near_value;
            }
            for(int k = -rad + 1;k < rad;k++){
                int idx = i + posGrid(-rad,k,w);
                if (data[idx] > 60) out[idx] = 100;
                else if(out[idx] < near_value)
                    out[idx] = near_value;
            }
            
        }
    }
    return nearnessMap;
}

vector<Vec<2>> path2Vec(const nav_msgs::Path &q){
    vector<Vec<2>> res;
    res.resize(q.poses.size());
    for (int i = 0; i < q.poses.size(); i++){
        auto &p = q.poses[i].pose.position;
        res[i][0] = p.x;
        res[i][1] = p.y;
    }
    return res;
}

void vec2Path(nav_msgs::Path &res, const vector<Vec<2>> &q){
    for (int i = 0; i < q.size(); i++){
        auto &p = res.poses[i].pose.position;
        p.x = q[i][0];
        p.y = q[i][1];
    }
}

Vec<2> localGradient(int index, const vector<Vec<2>> &orig, const vector<Vec<2>> &next, double a, double b){
    Vec<2> dist_gradient = zero< Vec<2> >();
    if(index > 0)
        dist_gradient += next[index] - next[index-1];
    if (index < orig.size() -1 && orig.size() > 0)
        dist_gradient += next[index] - next[index+1];
    return (next[index]-orig[index])*b + dist_gradient * a;
}

nav_msgs::Path PathSmoother::SmoothPath(nav_msgs::Path& path, float alpha, float beta)
{

    nav_msgs::Path newPath = path;    
    if(path.poses.size() < 3)
        return newPath;
    auto orig = path2Vec(path);
    auto next = orig;
    auto buffer = orig;
    auto size = orig.size();
    
    for(int l = 0;l < 70;l++){
        for(int i = 0;i < size;i++)
            buffer[i] = next[i] - localGradient(i,orig,next,alpha,beta)*0.5;
        next = buffer;
    }
    
    
    vec2Path(newPath,next);
    return newPath;
}
