#include <tuple>
using namespace std;
using Vector3 = std::tuple<double, double, double>;

// 輔助運算：向量相減
Vector3 sub(Vector3 a, Vector3 b);
// 輔助運算：內積 (Dot Product)
double dot(Vector3 a, Vector3 b);