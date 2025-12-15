#ifndef PATH_MANAGER_HPP
#define PATH_MANAGER_HPP

#include <vector>
#include <cmath>
#include <iostream>

// Estructura simple para un punto (X, Y, Z)
struct Point {
    double x, y, z;
};

class PathManager {
public:
    PathManager();

    // Intenta guardar un camino. Devuelve false si no cumple las reglas.
    bool setPath(const std::vector<Point>& points);

    // Simula la ejecución.
    bool executePath();

private:
    std::vector<Point> current_path_;
    bool is_valid_;

    // Aquí pondremos las reglas "falsas" para pasar el test
    bool validatePath(const std::vector<Point>& points);
};

#endif // PATH_MANAGER_HPP
