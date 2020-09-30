#include <benchmark/benchmark.h>
#include "collision_detect.h"

static void BM_check_polygon(benchmark::State& state) {
  CollisionDetect collision_detect;
  for (auto _ : state) {
    collision_detect.ComputeCollision();
  }
}
BENCHMARK(BM_check_polygon);


//static void BM_cv_picture(benchmark::State& state) {
//    CollisionDetect collision_detect;
//    for (auto _ : state) {
//        collision_detect.ComputeCollosion();
//    }
//}
//BENCHMARK(BM_cv_picture);

BENCHMARK_MAIN();