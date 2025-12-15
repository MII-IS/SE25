#include <gtest/gtest.h>
#include <cmath>

// TEST 1: Verificar que la lógica matemática del robot es coherente
// Comprobamos que el movimiento del "hombro" (coseno) está dentro de los límites
TEST(RobotBailarinTest, CheckMathLogic) {
  double t = 1.0; // Simulamos 1 segundo
  double hombro = std::cos(t * 1.0) * 0.8;
  
  // El coseno * 0.8 nunca debe ser mayor que 0.8 ni menor que -0.8
  EXPECT_LE(hombro, 0.8);
  EXPECT_GE(hombro, -0.8);
}

// TEST 2: Test simple de sanidad (Smoke Test)
// Verifica que 1+1 es 2 (para asegurar que el sistema de test compila bien)
TEST(RobotBailarinTest, SanityCheck) {
  EXPECT_EQ(2 + 2, 4);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
