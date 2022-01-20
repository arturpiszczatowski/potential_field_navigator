#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <vector>
#include <string>
#include <fstream>

/*
  MATH PORTION OF THE CODE
*/
using point2d = std::array<double, 2>;
using wall_cordinates = std::pair<point2d, point2d>;
inline point2d operator+(const point2d a, const point2d b)
{
    return {a[0] + b[0], a[1] + b[1]};
}
inline point2d operator*(const point2d a, const double b)
{
    return {a[0] * b, a[1] * b};
}
inline point2d operator-(const point2d a, const point2d b)
{
    return a + (b * -1.0);
}
inline point2d operator%(const point2d a, const point2d b)
{
    return {a[0] * b[0], a[1] * b[1]};
}
inline double operator*(const point2d a, const point2d b)
{
    auto r = a % b;
    return std::accumulate(r.begin(), r.end(), 0.0);
}
inline double length(const point2d a)
{
    return std::sqrt(a * a);
}
inline std::ostream& operator<<(std::ostream& o, const point2d a)
{
    o << a[0] << " " << a[1];
    return o;
}

point2d derivative(std::function<double(point2d)> f, point2d x, double d = 1.52588e-05)
{
    point2d dx = {d, 0.0};
    point2d dy = {0.0, d};
    return {
            (f(x + dx * 0.5) - f(x - dx * 0.5)) / d,
            (f(x + dy * 0.5) - f(x - dy * 0.5)) / d};
}

double minimal_distance(point2d pt1, point2d pt2, point2d pt3){

    // vector vw
    std::pair<double, double> pt1_pt2;
    pt1_pt2.first = pt2[0] - pt1[0];
    pt1_pt2.second = pt2[1] - pt1[1];

    // vector wp
    std::pair<double, double> pt1_pt3;
    pt1_pt3.first = pt3[0] - pt1[0];
    pt1_pt3.second = pt3[1] - pt1[1];

    // vector vp VP =1 VW= 2 wp =3
    std::pair<double, double> pt3_pt2;
    pt3_pt2.first = pt3[0] - pt2[0],
            pt3_pt2.second = pt3[1] - pt2[1];


    // Calculating the dot product
    double vw_wp = (pt1_pt2.first * pt3_pt2.first + pt1_pt2.second * pt1_pt3.second);
    double vw_vp = (pt1_pt2.first * pt1_pt3.first + pt1_pt2.second * pt1_pt3.second);

    // Minimum distance from
    // point p to the line segment
    double reqAns = 0.0;

    if (vw_wp > 0) {
        // Finding the magnitude
        double y = pt3[1] - pt2[1];
        double x = pt3[0] - pt2[0];
        reqAns = sqrt(x * x + y * y);
    } else if (vw_vp < 0) {
        double y = pt3[1] - pt1[1];
        double x = pt3[0] - pt1[0];
        reqAns = sqrt(x * x + y * y);
    } else {
        // Finding the perpendicular distance
        double x1 = pt1_pt2.first;
        double y1 = pt1_pt2.second;
        double x2 = pt1_pt3.first;
        double y2 = pt1_pt3.second;
        double mod = sqrt(x1 * x1 + y1 * y1);
        reqAns = abs(x1 * y2 - y1 * x2) / mod;
    }
    return reqAns;
}

int main(int argc, char** argv) {


    std::string results_file = "results.txt";
    std::ofstream out(results_file);
    point2d destination = {0.0, 0.0};
    point2d currentPosition = {10.0, 1.0};
    double velocity = 0.1;

//    std::pair<wall_cordinates, double> wall;
    std::vector<std::pair<wall_cordinates, double>> walls = {};

    walls.push_back({{{5.0, 1.0},{8.0, 3.0}}, (argc > 1) ? std::stod(argv[1]) : 0.005});
    walls.push_back({{{2.0, 0.0},{2.0,1.0}}, (argc > 1) ? std::stod(argv[1]) : 0.005});


    auto field = [&](point2d p) -> double {
        double obstacle_field = 0;
        for (const auto &wall: walls) {
            double distanceToObstacle = minimal_distance(wall.first.first, wall.first.second, p);
            obstacle_field += wall.second / (distanceToObstacle * distanceToObstacle);
        }
        return length(destination - p) + obstacle_field;
    };

    for (int i = 0; i < 200; i++) {
        point2d dp = derivative(field, currentPosition);
        dp = dp * (1.0 / length(dp));
        dp = dp * velocity;
        currentPosition = currentPosition - dp;
        out << currentPosition << std::endl;
    }


    for(const auto &wall : walls) {
        int k = 0;
        auto x1 = wall.first.first[0];
        auto y1 = wall.first.first[1];

        auto x2 = wall.first.second[0];
        auto y2 = wall.first.second[1];

        point2d wall_builder = {x1, y1};

        do{
            out << wall_builder << std::endl;

            if(wall_builder[0] > x2) {
                wall_builder[0] -= 0.1;
            }
            else if (wall_builder[0] < x2){
                wall_builder[0] += 0.1;
            } else {
                wall_builder[0]=x2;
            }

            if(wall_builder[1] > y2) {
                wall_builder[1] -= 0.1;
            }
            else if (wall_builder[1] < y2){
                wall_builder[1] += 0.1;
            } else {
                wall_builder[1]=y2;
            }

            k++;
        }while((wall_builder[0] != x2 || wall_builder[1] != y2) && k < 20);
    }
return 0;
}

