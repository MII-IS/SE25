#include "path_manager.hpp"

PathManager::PathManager() : is_valid_(false) {}

bool PathManager::setPath(const std::vector<Point>& points) {
    // Si la validación pasa, guardamos el camino
    if (validatePath(points)) {
        current_path_ = points;
        is_valid_ = true;
        return true;
    }
    // Si falla, marcamos como inválido
    is_valid_ = false;
    return false;
}

bool PathManager::executePath() {
    if (!is_valid_) {
        std::cerr << "Error: No se puede ejecutar un camino inválido.\n";
        return false;
    }
    // AQUÍ SIMULAMOS QUE SE MUEVE
    return true;
}

bool PathManager::validatePath(const std::vector<Point>& points) {
    // REGLA 1: El camino no puede estar vacío (TC-TRAJ-04)
    if (points.empty()) {
        std::cerr << "[Validation Fail] La lista de puntos está vacía.\n";
        return false;
    }

    // REGLA 2: Simulamos que el robot solo llega a 2 metros (TC-TRAJ-05)
    for (const auto& p : points) {
        double distance = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (distance > 2.0) {
            std::cerr << "[Validation Fail] Punto fuera del alcance (> 2.0m).\n";
            return false;
        }
    }

    return true;
}
