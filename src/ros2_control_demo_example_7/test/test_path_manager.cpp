#include <gtest/gtest.h>
#include "../src/path_manager.hpp" 

// TEST 1: Camino Válido (Happy Path)
TEST(PathManagerTest, TCTRAJ01_ValidSequence) {
    PathManager manager;
    std::vector<Point> valid_path = { {0.5, 0.0, 0.5}, {0.6, 0.1, 0.5} };

    bool accepted = manager.setPath(valid_path);
    bool executed = manager.executePath();

    EXPECT_TRUE(accepted) << "El sistema debería aceptar caminos válidos.";
    EXPECT_TRUE(executed) << "El sistema debería ejecutar caminos válidos.";
}

// TEST 2: Camino de un solo punto (Boundary)
TEST(PathManagerTest, TCTRAJ02_SinglePointSequence) {
    PathManager manager;
    std::vector<Point> single_point = { {0.5, 0.0, 0.5} };
    bool accepted = manager.setPath(single_point);
    EXPECT_TRUE(accepted) << "El sistema debería aceptar un solo punto.";
}

// TEST 3: Camino Vacío (Defecto/Safety)
TEST(PathManagerTest, TCTRAJ04_EmptySequence) {
    PathManager manager;
    std::vector<Point> empty_path; 
    bool accepted = manager.setPath(empty_path);
    EXPECT_FALSE(accepted) << "El sistema DEBE rechazar listas vacías.";
}

// TEST 4: Punto inalcanzable (Defecto)
TEST(PathManagerTest, TCTRAJ05_UnreachableTarget) {
    PathManager manager;
    std::vector<Point> far_path = { {5.0, 0.0, 0.0} };
    bool accepted = manager.setPath(far_path);
    EXPECT_FALSE(accepted) << "El sistema DEBE rechazar puntos lejanos.";
}
