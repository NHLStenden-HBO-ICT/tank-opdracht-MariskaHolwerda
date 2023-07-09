#include "precomp.h"
#include "terrain.h"
#include <queue>
#include <cmath>



namespace fs = std::filesystem;
namespace Tmpl8
{
    Terrain::Terrain()
    {
        //Load in terrain sprites
        grass_img = std::make_unique<Surface>("assets/tile_grass.png");
        forest_img = std::make_unique<Surface>("assets/tile_forest.png");
        rocks_img = std::make_unique<Surface>("assets/tile_rocks.png");
        mountains_img = std::make_unique<Surface>("assets/tile_mountains.png");
        water_img = std::make_unique<Surface>("assets/tile_water.png");


        tile_grass = std::make_unique<Sprite>(grass_img.get(), 1);
        tile_forest = std::make_unique<Sprite>(forest_img.get(), 1);
        tile_rocks = std::make_unique<Sprite>(rocks_img.get(), 1);
        tile_water = std::make_unique<Sprite>(water_img.get(), 1);
        tile_mountains = std::make_unique<Sprite>(mountains_img.get(), 1);


        //Load terrain layout file and fill grid based on tiletypes
        fs::path terrain_file_path{ "assets/terrain.txt" };
        std::ifstream terrain_file(terrain_file_path);

        if (terrain_file.is_open())
        {
            std::string terrain_line;

            std::getline(terrain_file, terrain_line);
            std::istringstream lineStream(terrain_line);

            int rows;

            lineStream >> rows;

            for (size_t row = 0; row < rows; row++)
            {
                std::getline(terrain_file, terrain_line);

                for (size_t collumn = 0; collumn < terrain_line.size(); collumn++)
                {
                    switch (std::toupper(terrain_line.at(collumn)))
                    {
                    case 'G':
                        tiles.at(row).at(collumn).tile_type = TileType::GRASS;
                        break;
                    case 'F':
                        tiles.at(row).at(collumn).tile_type = TileType::FORREST;
                        break;
                    case 'R':
                        tiles.at(row).at(collumn).tile_type = TileType::ROCKS;
                        break;
                    case 'M':
                        tiles.at(row).at(collumn).tile_type = TileType::MOUNTAINS;
                        break;
                    case 'W':
                        tiles.at(row).at(collumn).tile_type = TileType::WATER;
                        break;
                    default:
                        tiles.at(row).at(collumn).tile_type = TileType::GRASS;
                        break;
                    }
                }
            }
        }
        else
        {
            std::cout << "Could not open terrain file! Is the path correct? Defaulting to grass.." << std::endl;
            std::cout << "Path was: " << terrain_file_path << std::endl;
        }

        //Instantiate tiles for path planning
                
        for (size_t y = 0; y < tiles.size(); y++)
        {
            for (size_t x = 0; x < tiles.at(y).size(); x++)
            {
                tiles.at(y).at(x).position_x = x;
                tiles.at(y).at(x).position_y = y;

                if (is_accessible(y, x + 1)) { tiles.at(y).at(x).exits.push_back(&tiles.at(y).at(x + 1)); }
                if (is_accessible(y, x - 1)) { tiles.at(y).at(x).exits.push_back(&tiles.at(y).at(x - 1)); }
                if (is_accessible(y + 1, x)) { tiles.at(y).at(x).exits.push_back(&tiles.at(y + 1).at(x)); }
                if (is_accessible(y - 1, x)) { tiles.at(y).at(x).exits.push_back(&tiles.at(y - 1).at(x)); }
            }
        }
    }

    void Terrain::update()
    {
        //Pretend there is animation code here.. next year :)
    }

    void Terrain::draw(Surface* target) const
    {

        for (size_t y = 0; y < tiles.size(); y++)
        {
            for (size_t x = 0; x < tiles.at(y).size(); x++)
            {
                int posX = (x * sprite_size) + HEALTHBAR_OFFSET;
                int posY = y * sprite_size;

                switch (tiles.at(y).at(x).tile_type)
                {
                case TileType::GRASS:
                    tile_grass->draw(target, posX, posY);
                    break;
                case TileType::FORREST:
                    tile_forest->draw(target, posX, posY);
                    break;
                case TileType::ROCKS:
                    tile_rocks->draw(target, posX, posY);
                    break;
                case TileType::MOUNTAINS:
                    tile_mountains->draw(target, posX, posY);
                    break;
                case TileType::WATER:
                    tile_water->draw(target, posX, posY);
                    break;
                default:
                    tile_grass->draw(target, posX, posY);
                    break;
                }
            }
        }
    }

    
    // Use A* search to find the shortest route to the destination

  // ***** Define a struct for the A* search nodes *****
    struct AStarNode {
        float position_x;
        float position_y;
        float g_score; 
        float f_score;
        AStarNode* came_from;

        AStarNode(float x, float y, float g, float f, AStarNode* prev) : position_x(x), position_y(y), g_score(g), f_score(f), came_from(prev) {}
    };

    // ***** Define a comparator for the priority queue in the A* algorithm *****
    struct AStarNodeComparator {
        bool operator()(const AStarNode* a, const AStarNode* b) const {
            return a->f_score < b->f_score;
        }
    };

    // Heuristic function to estimate the cost from a node to the goal 
    float Terrain::heuristic(float currentX1, float currentY1, float exitX2, float exitY2) {
        float goalX = std::abs(exitX2 - currentX1);
        float goalY = std::abs(exitY2 - currentY1);
        return goalX + goalY;  // Manhattan distance, omdat met het berekenen van de wortel van de som, het programma vastliep
    }

    // ***** quicksort functionality
	// Swap two vec2 elements
    void swap(vec2& a, vec2& b) {
        vec2 temp = a;
        a = b;
        b = temp;
    }

    // Partition the vector and return the pivot index
    int partition(std::vector<vec2>& arr, int low, int high) {
        vec2 pivot = arr[high];
        int i = low - 1;

        for (int j = low; j <= high - 1; j++) {
            if (arr[j].x <= pivot.x) {                       
                i++;
                swap(arr[i], arr[j]);
            }
        }

        swap(arr[i + 1], arr[high]);
        return i + 1;
    }

    // Quicksort recursive functionality
    void quicksort(std::vector<vec2>& arr, int low, int high) {
        if (low < high) {
            int pivot = partition(arr, low, high);

            quicksort(arr, low, pivot - 1);
            quicksort(arr, pivot + 1, high);
        }
    }

    // ***** quicksort functionality end

    std::vector<vec2> Terrain::get_route(const Tank& tank, const vec2& target) {
        const size_t pos_x = tank.position.x / sprite_size;
        const size_t pos_y = tank.position.y / sprite_size;

        const size_t target_x = target.x / sprite_size;
        const size_t target_y = target.y / sprite_size;

        std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNodeComparator> queue;
        AStarNode* start_node = new AStarNode(pos_x, pos_y, 0.0f, 0.0f, nullptr);
        queue.push(start_node);

        std::vector<std::vector<bool>> visited(tiles.size(), std::vector<bool>(tiles[0].size(), false));

        bool route_found = false;
        AStarNode* current_node = nullptr;

        while (!queue.empty() && !route_found) {
            current_node = queue.top();
            queue.pop();

            if (current_node->position_x == target_x && current_node->position_y == target_y) {
                route_found = true;
                break;
            }

            visited[current_node->position_y][current_node->position_x] = true;



            for (TerrainTile* exit : tiles[current_node->position_y][current_node->position_x].exits) {
                if (visited[exit->position_y][exit->position_x]){
                    continue;
                }

                float new_g_score = current_node->g_score + 1;// current_node->g_score + heuristic(current_node->position_x, current_node->position_y, exit->position_x, exit->position_y);
                float new_f_score = new_g_score + heuristic(exit->position_x, exit->position_y, target_x, target_y);


                AStarNode* neighbor = new AStarNode(exit->position_x, exit->position_y, new_g_score, new_f_score, current_node);
                queue.push(neighbor);
            }
        }

        std::vector<vec2> route;
 

        if (route_found) {
            while (current_node != nullptr) {
                route.push_back(vec2((float)current_node->position_x * sprite_size, (float)current_node->position_y * sprite_size));
                current_node = current_node->came_from;
            }

            std::reverse(route.begin(), route.end());
            quicksort(route, 0, route.size() - 1);
            
        }
     

        while (!queue.empty()) {
            delete queue.top();
            queue.pop();
        }

        return route;
    }

    

    /*
          
    //Use Breadth-first search to find shortest route to the destination
    vector<vec2> Terrain::get_route(const Tank& tank, const vec2& target)
    {
        //Find start and target tile
        const size_t pos_x = tank.position.x / sprite_size;
        const size_t pos_y = tank.position.y / sprite_size;

        const size_t target_x = target.x / sprite_size;
        const size_t target_y = target.y / sprite_size;

        //Init queue with start tile
        std::queue<vector<TerrainTile*>> queue;
        queue.emplace();
        queue.back().push_back(&tiles.at(pos_y).at(pos_x));

        std::vector<TerrainTile*> visited;

        bool route_found = false;
        vector<TerrainTile*> current_route;
        while (!queue.empty() && !route_found)
        {
            current_route = queue.front();
            queue.pop();
            TerrainTile* current_tile = current_route.back();

            //Check all exits, if target then done, else if unvisited push a new partial route
            for (TerrainTile * exit : current_tile->exits)
            {
                if (exit->position_x == target_x && exit->position_y == target_y)
                {
                    current_route.push_back(exit);
                    route_found = true;
                    break;
                }
                else if (!exit->visited)
                {
                    exit->visited = true;
                    visited.push_back(exit);
                    queue.push(current_route);
                    queue.back().push_back(exit);
                }
            }
        }

        //Reset tiles
        for (TerrainTile * tile : visited)
        {
            tile->visited = false;
        }

        if (route_found)
        {
            //Convert route to vec2 to prevent dangling pointers
            std::vector<vec2> route;
            for (TerrainTile* tile : current_route)
            {
                route.push_back(vec2((float)tile->position_x * sprite_size, (float)tile->position_y * sprite_size));
            }

            return route;
        }
        else
        {
            return  std::vector<vec2>();
        }

    }
    

    */

    //TODO: Function not used, convert BFS to dijkstra and take speed into account next year :)
    float Terrain::get_speed_modifier(const vec2& position) const
    {
        const size_t pos_x = position.x / sprite_size;
        const size_t pos_y = position.y / sprite_size;

        switch (tiles.at(pos_y).at(pos_x).tile_type)
        {
        case TileType::GRASS:
            return 1.0f;
            break;
        case TileType::FORREST:
            return 0.5f;
            break;
        case TileType::ROCKS:
            return 0.75f;
            break;
        case TileType::MOUNTAINS:
            return 0.0f;
            break;
        case TileType::WATER:
            return 0.0f;
            break;
        default:
            return 1.0f;
            break;
        }
    }

    bool Terrain::is_accessible(int y, int x)
    {
        //Bounds check
        if ((x >= 0 && x < terrain_width) && (y >= 0 && y < terrain_height))
        {
            //Inaccessible terrain check
            if (tiles.at(y).at(x).tile_type != TileType::MOUNTAINS && tiles.at(y).at(x).tile_type != TileType::WATER)
            {
                return true;
            }
        }

        return false;
    }
}