#include <queue>
#include <iostream>
#include <array>
#include <set>
#include <memory>
#include <algorithm>
#include <math.h>       /* round, floor, ceil, trunc */

#include "uahrk_navigation/GridNode.hpp"



struct Move{
    int x_increment;
    int y_increment;
    Move() : x_increment{0}, y_increment{0} {};
    Move(int _x, int _y) : x_increment{_x}, y_increment{_y} {};
    friend inline bool operator==(const Move& lhs, const Move& rhs) { 
        return (lhs.x_increment == rhs.x_increment && lhs.y_increment == rhs.y_increment); 
    }
    friend inline bool operator!=(const Move& lhs, const Move& rhs) {
        return !(lhs == rhs);
    }
    Move& operator=(Move other)
    {
        x_increment = other.x_increment;
        y_increment = other.y_increment;
        return *this;
    }
};

struct Cell
{
    unsigned int x;
    unsigned int y;
    Cell() : x{0} , y{0} {};
    Cell(const unsigned int _x, const unsigned int _y) : x{_x}  , y{_y}  {};

    friend inline bool operator==(const Cell& lhs, const Cell& rhs) { 
        return (lhs.x == rhs.x && lhs.y == rhs.y); 
    }

    friend inline bool operator!=(const Cell& lhs, const Cell& rhs) {
        return !(lhs == rhs);
    }
};


struct AstarCell : Cell {
    unsigned int cost;
    unsigned int function;

    std::shared_ptr<AstarCell> father;


    AstarCell() : Cell{0, 0}, cost{0}, function{0} {};
    AstarCell(const Cell _cell) : Cell{_cell.x, _cell.y}, cost{0}, function{0} {};

    AstarCell(const Cell _cell, const unsigned int _cost, const unsigned int _function) : Cell{_cell.x, _cell.y}, cost{_cost}, function{_function} {};
    AstarCell(const unsigned int _x, const unsigned int _y) : Cell{_x,_y}, cost{0}, function{0}, father{nullptr} {};
    AstarCell(const unsigned int _x, const unsigned int _y, AstarCell _father) : Cell{_x,_y}, cost{0}, function{0}, father{std::make_shared<AstarCell>(_father)} {};

    AstarCell(const unsigned int _x, const unsigned int _y, const unsigned int _cost, const unsigned int _function) : Cell{_x,_y}, cost{_cost}, function{_function} {};

    friend inline bool operator==(const AstarCell& lhs, const AstarCell& rhs) { 
        return (lhs.x == rhs.x && lhs.y == rhs.y); 
    }
    friend inline bool operator!=(const AstarCell& lhs, const AstarCell& rhs) {
        return !(lhs == rhs);
    }
    friend inline bool operator<(const AstarCell& lhs, const AstarCell& rhs) {
        //std::cout << "lhs " << lhs.function << " rhs " << rhs.function << std::endl;
        return (lhs.function > rhs.function);
    }

    /*

    friend inline bool operator<=(const AstarCell& lhs, const AstarCell& rhs) {
        return (lhs.function >= rhs.function);
    }

    friend inline bool operator>(const AstarCell& lhs, const AstarCell& rhs) {
        return (lhs.function < rhs.function);
    }
    friend inline bool operator>=(const AstarCell& lhs, const AstarCell& rhs) {
        return (lhs.function <= rhs.function);
    }
    */

};

template<unsigned int w, unsigned h>RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
struct Grid{
    unsigned int width  = w;
    unsigned int height = h;
    unsigned int cells[w][h] {0};
};

unsigned int square_euclidean_dist(const AstarCell p1, const AstarCell p2){
    unsigned int dx = p2.x - p1.x;
    unsigned int dy = p2.y - p1.y;
    return (dx*dx) + (dy*dy);
}


inline bool cell_in_grid(Cell  cell, Grid<EurobotGridWidth,EurobotGridHeight> grid){
    return (0 <= cell.x && cell.x < grid.width && 0 <= cell.y && cell.y < grid.height);
}

inline bool cell_ocupied(Cell  cell, Grid<EurobotGridWidth,EurobotGridHeight> grid){
    return grid.cells[cell.x][cell.y];
}


//Move[2] MOVES = {Move(1,0) , Move(1,1)};

std::array<Move,8> MOVES = {Move( 1 ,  0), 
                            Move( 1 ,  1),
                            Move( 0 ,  1),
                            Move(-1 ,  1),
                            Move(-1 ,  0),
                            Move(-1 , -1),
                            Move( 0 , -1),
                            Move( 1 , -1)} ;


std::vector<AstarCell> explore_node(AstarCell &node, const Grid<EurobotGridWidth,EurobotGridHeight> &grid){
    
    std::vector<AstarCell> new_nodes;
    
    // Iterar sobre los movimientos 
    for(const auto & move: MOVES){
        // Calcular la posición del nodo vecino
        AstarCell new_cell = {node.x + move.x_increment, node.y + move.y_increment, node};
        // Comprobar si el nodo no esta dentro del grid
        if (!cell_in_grid(new_cell,grid)) continue;     // Si no esta dentro del grid descartarlo

        // Si el nodo está ocupado descartarlo
        if (cell_ocupied(new_cell,grid)) continue;  

        // Añadir los nodos a la lista de vectores
        new_nodes.push_back(new_cell);
    }
    return new_nodes;
}


std::vector<Cell> calculate_path(const AstarCell &last_cell){

    std::vector<Cell> path;
    AstarCell actual_cell = last_cell;

    while (actual_cell.father != nullptr){
        path.emplace_back(actual_cell.x, actual_cell.y);
        actual_cell = *actual_cell.father;
    }
    path.emplace_back(actual_cell.x, actual_cell.y);
    std::reverse(path.begin(), path.end());
    return path;
}



std::vector<Cell> Astar(const AstarCell &start_cell, const AstarCell &goal_cell ,const Grid<EurobotGridWidth,EurobotGridHeight> &grid){
    
    // If the end cell is ocupied return quickly as posible
    if (cell_ocupied(goal_cell, grid))  throw("No Astar solution");
    
    std::set<AstarCell> visited_cells;
    std::priority_queue<AstarCell> discovered_cells;
    AstarCell best_node;
    discovered_cells.push(start_cell);
    int n = 0;
    // Mientras halla nodos que se puedan ha 
    while(!discovered_cells.empty()){
        // Obtener el mejor nodo
        n++;
        best_node = discovered_cells.top();
        discovered_cells.pop();

        // Añadirlo a los nodos visitados
        visited_cells.insert(best_node);

        // Crear nodos (calculando su heuristica)
        auto new_nodes = explore_node(best_node,grid);

        // Por cada nodo nuevo
        for(auto &new_node : new_nodes){
            // Comprobar si el nodo es el nodo final:
            if (new_node == goal_cell){   
                return calculate_path(new_node);
            }else{
                // Calculate heuristic
                auto heuristic = square_euclidean_dist(new_node, goal_cell);
                
                // Update new node path cost
                new_node.cost = best_node.cost + 1;

                // Update node function cost
                new_node.function = heuristic + new_node.cost;

                // Añadirlo a los nodos por explorar
                discovered_cells.push(new_node);
            }
        }
    }

    // Retornar error
    throw("No Astar solution");
}

inline Move Calculate_Increment(const Cell &start, const Cell &end){
    return {end.x - start.x, end.y - start.y};
}



std::vector<Cell> DiscretizePath(std::vector<Cell> path){
    
    // Safety check
    if(path.size() <= 1) return path;

    // 
    std::vector<Cell> discretized_path;

    // Calculate the head move
    Move last_move = Calculate_Increment(path[0], path[1]);

    Cell last_wp = path[1];
    Move actual_move;

    for (auto wp = begin(path) + 2; wp != end (path); ++wp) {
        actual_move = Calculate_Increment(last_wp,*wp);
        if(actual_move != last_move){
            discretized_path.push_back(last_wp);
        }
        last_move = actual_move;
        last_wp = *wp ;
    }

    // Careful with the last element
    discretized_path.push_back(last_wp);
    return discretized_path;
}

bool safe_conection(const Cell &start,const Cell &end,const Grid<EurobotGridWidth,EurobotGridHeight> &grid){

    if (cell_ocupied(start,grid)) return false;

    auto increment = Calculate_Increment(start, end);
    constexpr float error_float  = 0.1;
    Cell wp           = start;
    float x_increment = 0;
    float y_increment = 0;
    float x           = start.x;
    float y           = start.y;
    float cnt         = 0.0;
    float slope       = 0.0;


    if (abs(increment.x_increment) >= abs(increment.y_increment)){
        // Careful dividing by zero
        float slope = (float)increment.y_increment / (float)increment.x_increment;

        // CUIDADO CON LOS SIGNOS
        if(increment.x_increment > 0) x_increment = 1.0;
        else                          x_increment = -1.0;
        if(increment.y_increment > 0){
            y_increment = abs(slope);  
            std::cout << "hola " << std::endl;
        }
        else                          y_increment = -abs(slope);

        std::cout << "slope " << slope << " x_increment  " << x_increment << " y_increment " << y_increment << std::endl;

    }else{
        // CUIDADO CON LOS SIGNOS
        if(increment.x_increment > 0) x_increment = 1.0;
        else                          x_increment = -1.0;
        if(increment.y_increment > 0){
            y_increment = abs(slope);  
            std::cout << "hola " << std::endl;
        }
        else                          y_increment = -abs(slope);

        std::cout << "slope " << slope << " x_increment  " << x_increment << " y_increment " << y_increment << std::endl;

    }


    std::cout << "x " << x << " y " << y << std::endl;
    std::cout << "wp  (" << wp.x << ", " << wp.y << ") "  << std::endl;


    y += 0.5 * y_increment;
    wp.y = round(y);
    if (cell_ocupied(wp,grid)) return false;

    //wp.x = round(x);

    x += 0.5 * x_increment;
    std::cout << "x " << x << " y " << y << std::endl;
    std::cout << "wp  (" << wp.x << ", " << wp.y << ") "  << std::endl;


    while (wp != end){
        x += x_increment;
        wp.x = x;
        std::cout << "x " << x << " y " << y << std::endl;
        std::cout << "wp  (" << wp.x << ", " << wp.y << ") "  << std::endl;

        if (cell_ocupied(wp,grid)) return false;
        if (wp == end)             return true;

        y += y_increment;
        wp.y = round(y);
        if (cell_ocupied(wp,grid)) return false;
        std::cout << "x " << x << " y " << y << std::endl;
        std::cout << "wp  (" << wp.x << ", " << wp.y << ") "  << std::endl;
    }
    return true;
}


        /*
        // Compruebo celda
        if(cell_ocupied(wp,grid)) return false;
        if(wp == end)  return true;
        cnt += 1.0;

        std::cout << "wp  (" << wp.x << ", " << wp.y << ") "  << std::endl;


        // Si el contardor llega al valor de pendiente o a uno mayor
        if(cnt + 0.5 >= slope){
            // Añado 1 a las ys
            wp.y += y_increment;
            std::cout << "cnt wp  (" << wp.x << ", " << wp.y << ") "  << std::endl;

            cnt = cnt - slope;
            // Compruebo celda;
            if(cell_ocupied(wp,grid)) return false;
        }
        */


std::vector<Cell> reduce_by_slopes(std::vector<Cell> path, const Grid<EurobotGridWidth,EurobotGridHeight> &grid){

        
    // Safety check
    if(path.size() <= 1) return path;

    auto start_wp = path.begin();


    for(auto wp = end(path) - 1;  wp != start_wp + 1; --wp) {
        // Hay que comprobar si hay pendiente también
        if (safe_conection(*start_wp,*wp, grid)) {
            // Eliminar todos los wps que hay entre start_wp y wp
            // Repetir el proceso para los puntos posteriores hasta que llegues
            // al penúltimo punto
            break;
        }
    } 



}



int main(){
    Grid<EurobotGridWidth,EurobotGridHeight> grid;
    AstarCell start(5,5);
    AstarCell goal(11,9);
    auto path = Astar(start, goal,  grid);
    std::cout << "El camino encontrado es " << std::endl;
    //for(auto n: path){ 
    //    std::cout << "nodo (" << n.x << ", " << n.y << ") "  << std::endl;
    //}   
    auto discretized_path = DiscretizePath(path);

    std::cout << "El camino discretizado " << std::endl;
    //for(auto n: discretized_path){ 
    //    std::cout << "nodo (" << n.x << ", " << n.y << ") "  << std::endl;
    //}   
    // Casos en el power point 
    safe_conection({1,0},{0,1},grid);


    return 0;
}