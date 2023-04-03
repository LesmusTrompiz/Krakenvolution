#include "uahrk_navigation/astar_ros.hpp"
#include <array>

unsigned int square_euclidean_dist(const AstarCell p1, const AstarCell p2, const nav_msgs::msg::OccupancyGrid &grid){
    int p1x = p1.cell % grid.info.width;
    int p1y = (p1.cell - p1x) / grid.info.width;
    int p2x = p2.cell % grid.info.width;
    int p2y = (p2.cell - p2x) / grid.info.width;
    unsigned int dx = p2x - p1x;
    unsigned int dy = p2y - p1y;
    return (dx*dx) + (dy*dy);
}

//inline bool cell_in_grid(const AstarCell  &cell, const nav_msgs::msg::OccupancyGrid &grid){
//    return 0 <= cell.cell && cell.cell < grid.info.height;
//}

inline bool cell_ocupied(const AstarCell &cell, const nav_msgs::msg::OccupancyGrid &grid){
    return grid.data[cell.cell]!=0;
}
inline bool cell_ocupied(const int &cell, const nav_msgs::msg::OccupancyGrid &grid){
    return grid.data[cell]!=0;
}
void printcell(int cell, const nav_msgs::msg::OccupancyGrid &grid){
    int x;
    int y;
    x = cell % grid.info.width;
    y = (cell - x) / grid.info.width;
    std::cout << "Cell id: " << cell << " x " << x << " y " << y << std::endl;
    return;
}


std::vector<AstarCell> explore_node(AstarCell &node, const std::array<int,8> &moves, const nav_msgs::msg::OccupancyGrid &grid){
   
    std::vector<AstarCell> new_nodes;
    int x = node.cell % grid.info.width;
    int y = (node.cell - x) / grid.info.width;
    int max_cell = grid.info.width * grid.info.height;


    // Iterar sobre los movimientos 
    for(const auto & move: moves){
        // Calcular la posición del nodo vecino
        AstarCell new_cell = {node.cell + move, node};
        
        // Checkeo de que esta dentro del grid
        int x_diff = new_cell.cell % grid.info.width - x;
        if( x_diff > 1 || x_diff < -1)                     continue;
        if(new_cell.cell < 0 || new_cell.cell >= max_cell) continue;
        
        // Si el nodo está ocupado descartarlo
        if (cell_ocupied(new_cell,grid)) continue;  

        // Añadir los nodos a la lista de vectores
        new_nodes.push_back(new_cell);
    }
    return new_nodes;
}


std::vector<int> calculate_path(const AstarCell &last_cell){
    std::vector<int> path;
    AstarCell actual_cell = last_cell;

    while (actual_cell.father != nullptr){
        path.emplace_back(actual_cell.cell);
        actual_cell = *actual_cell.father;
    }
    path.emplace_back(actual_cell.cell);
    std::reverse(path.begin(), path.end());
    return path;
}




std::vector<int> Astar(const AstarCell &start, const AstarCell &goal_cell, const nav_msgs::msg::OccupancyGrid &grid){
    
    // If the end cell is ocupied return quickly as posible
    if (cell_ocupied(start,grid)){
        throw(1);
    }    

    if (cell_ocupied(goal_cell,grid)){
        std::cout << "celda start ocupada >:(" << std::endl;
        throw(2);
    }    

    std::array<int,8> moves = {1,  
                              1 + (int)grid.info.width, 
                              (int)grid.info.width, 
                              -1 + (int)grid.info.width,
                              -1,
                              -1 - (int)grid.info.width, 
                              -(int)grid.info.width, 
                              1 - (int)grid.info.width};

    std::set<int> visited_cells;
    std::priority_queue<AstarCell> discovered_cells;
    AstarCell best_node;
    discovered_cells.push({start.cell});
    int n = 0;
    // Mientras halla nodos que se puedan ha 
    while(!discovered_cells.empty() || n < 500){
        // Obtener el mejor nodo
        n++;
        best_node = discovered_cells.top();
        discovered_cells.pop();

        if(visited_cells.find(best_node.cell) != visited_cells.end()){
            std::cout << "holi";
            continue;

        }

        // Añadirlo a los nodos visitados
        visited_cells.insert(best_node.cell);

        // Crear nodos (calculando su heuristica)
        auto new_nodes = explore_node(best_node,moves,grid);

        // Por cada nodo nuevo
        for(auto &new_node : new_nodes){
            // Comprobar si el nodo es el nodo final:
            if (new_node == goal_cell){
                return calculate_path(new_node);
            }else{
                if(visited_cells.find(new_node.cell) != visited_cells.end()){
                    continue;
                }else{
                    
                    // Calculate heuristic
                    auto heuristic = square_euclidean_dist(new_node, goal_cell, grid);
                    
                    // Update new node path cost
                    new_node.cost = best_node.cost + 1;

                    // Update node function cost
                    new_node.function = heuristic + new_node.cost;

                    // Añadirlo a los nodos por explorar
                    discovered_cells.push(new_node);
                }
            }
        }
    }
    std::cout << "TU QUE NO HAY SOLUCIÓN" << std::endl;
    // Retornar error
    throw(3);
}

int pose2dintcell(Pose2d p, const nav_msgs::msg::OccupancyGrid &grid){
    int x = (p.x / grid.info.resolution);
    int y = (p.y / grid.info.resolution) ;
    y    *= grid.info.width;
    return x + y;
}

Pose2d intcell2pose2d(const int cell, const nav_msgs::msg::OccupancyGrid &grid){
    Pose2d p;
    p.x = cell % grid.info.width;
    p.y = (cell - p.x) / grid.info.width;
    return p;
}

geometry_msgs::msg::Pose intcell2rospose(const int cell, const nav_msgs::msg::OccupancyGrid &grid){
    geometry_msgs::msg::Pose p;
    p.position.x = cell % grid.info.width;
    p.position.y = (cell - p.position.x) / grid.info.width;
    p.position.x = p.position.x * grid.info.resolution;
    p.position.y = p.position.y * grid.info.resolution;
    return p;
}


geometry_msgs::msg::PoseArray intpath2rospath(std::vector<int> intpath, const nav_msgs::msg::OccupancyGrid &grid){
    //
    geometry_msgs::msg::PoseArray rospath;
    rospath.header.frame_id = "map";
    
    //
    for (const auto &int_cell : intpath){
        rospath.poses.push_back(intcell2rospose(int_cell,grid));
    }
    return rospath;
}


// /**/
// inline Move Calculate_Increment(const Cell &start, const Cell &end){
//     return {end.x - start.x, end.y - start.y};
// }



std::vector<int> discretize_path(std::vector<int> path){
    
    // Safety check
    if(path.size() <= 1) return path;

    std::vector<int> discretized_path {path[0]};

    // Calculate the head move
    int last_increment =  path[1] - path[0];

    int last_wp = path[1];
    int actual_increment;

    for (auto wp = begin(path) + 2; wp != end (path); ++wp) {
        actual_increment = *wp - last_wp;
        if(actual_increment != last_increment){
            discretized_path.push_back(last_wp);
        }
        last_increment = actual_increment;
        last_wp = *wp;
    }

    // Careful with the last element
    discretized_path.push_back(last_wp);
    return discretized_path;
}

Pose2d calculate_slope(Pose2d start, Pose2d end){
    Pose2d increment {end.x - start.x, end.y - start.y};
    Pose2d slope;

    if(abs(increment.x) >= abs(increment.y)){
        // Careful dividing by zero
        float m = (float)increment.y / (float)increment.x;

        // CUIDADO CON LOS SIGNOS
        if(increment.x > 0)           slope.x =  1.0;
        else                          slope.x = -1.0;
        if(increment.y > 0)           slope.y =  abs((float)increment.y / (float)increment.x);  
        else if(increment.y < 0)      slope.y = -abs((float)increment.y / (float)increment.x);
        else                          slope.y =  0.0;
    }else{
        if(increment.y > 0)           slope.y =  1.0;
        else                          slope.y = -1.0;
        if(increment.x > 0)           slope.x =  abs((float)increment.x / (float)increment.y);  
        else if(increment.x < 0)      slope.x = -abs((float)increment.x / (float)increment.y);
        else                          slope.x =  0.0;
    }
    return slope;
}

bool safe_conection(const int &start,const int &end, const nav_msgs::msg::OccupancyGrid &grid){

    if (cell_ocupied(start,grid)) return false;

    float wp          = start;
    auto  increment   = calculate_slope(intcell2pose2d(start,grid),intcell2pose2d(end,grid));
    std::cout << "increment x " << increment.x << "increment y " << increment.y << std::endl;
    
    float x_increment = 0;
    float y_increment = 0;
    auto e = intcell2pose2d(end,grid);
    
    std::cout << "end goal " << e.x << " " << e.y << std::endl;


    int   cnt  = 0;
    while (cnt < 300){
        auto p = intcell2pose2d(wp,grid);
        //std::cout << "wp after y increment " << p.x << " " << p.y << std::endl;
        x_increment += increment.x; 
        wp = start + round(x_increment) + round(y_increment) * grid.info.width;
        
        p = intcell2pose2d(wp ,grid);
        //std::cout << "wp after x increment" << p.x << " " << p.y << std::endl;
        if (cell_ocupied(wp,grid)) return false;
        if (wp == end)             return true;
        cnt++;

        y_increment += increment.y; 
        wp = start + round(x_increment) + round(y_increment) * grid.info.width;
        if (cell_ocupied(wp,grid)) return false;
        if (wp == end)             return true;
        cnt++;
    }
    std::cout << "Max count reached " << std::endl;
    return false;
}




std::vector<int> reduce_by_slopes(std::vector<int> path, const nav_msgs::msg::OccupancyGrid &grid){
     
    std::vector<int> reduced_path;

    // Safety check
    if(path.size() <= 2) return path;


    auto start_wp = path.begin();
    
    for(start_wp = path.begin(); start_wp != path.end() - 1; ++start_wp){
        reduced_path.push_back(*start_wp);
        std::cout << "Add start wp " << *start_wp << std::endl;

        for(auto wp = path.end() - 1;  wp != start_wp + 1; --wp) {
            // Hay que comprobar si hay pendiente también
            std::cout << "Is a safe conection between " << *start_wp << " and " << *wp << std::endl;
            if (safe_conection(*start_wp,*wp, grid)) {
                std::cout << "Safe conection " << std::endl;
                // Eliminar todos los wps que hay entre start_wp y wp
                // Repetir el proceso para los puntos posteriores hasta que llegues
                // al penúltimo punto
                start_wp = wp -1;
                std::cout << "start wp value changed to " << *start_wp << std::endl; 
                break;
            }
            std::cout << "NOT Safe conection " << std::endl;

        } 
    }
    std::cout << "Add last wp " << *start_wp << std::endl;
    reduced_path.push_back(*start_wp);
    return reduced_path;
}



// int main(){
//     Grid<EurobotGridWidth,EurobotGridHeight> grid;
//     AstarCell start(5,5);
//     AstarCell goal(11,9);
//     auto path = Astar(start, goal,  grid);
//     std::cout << "El camino encontrado es " << std::endl;
//     //for(auto n: path){ 
//     //    std::cout << "nodo (" << n.x << ", " << n.y << ") "  << std::endl;
//     //}   
//     auto discretized_path = DiscretizePath(path);

//     std::cout << "El camino discretizado " << std::endl;
//     //for(auto n: discretized_path){ 
//     //    std::cout << "nodo (" << n.x << ", " << n.y << ") "  << std::endl;
//     //}   
//     // Casos en el power point 
//     safe_conection({1,0},{0,1},grid);


//     return 0;
// }