#define _USE_MATH_DEFINES
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <random>
using namespace std;
namespace INFOGA_Bonus_CPP
{
    struct Point
    {
        double X, Y;
        Point(double x, double y) : X(x), Y(y) {}
    };

    class GrahamScan
    {
        static constexpr double epsilon = 1e-9;
        // The algorithm is clockwise
        static vector<Point> GetConvexHull(vector<Point> &points)
        {
            // Remove duplicates to prevent division zero errors
            // We sort first solely for removing duplicates here.
            // O(n log n)
            sort(points.begin(), points.end(), [](const Point &a, const Point &b)
                 {
                    if (abs(a.X - b.X) > epsilon) return a.X < b.X;
                    return a.Y < b.Y - epsilon; });
            auto last = unique(points.begin(), points.end(), [](const Point &a, const Point &b)
                               { return abs(a.X - b.X) < epsilon && abs(a.Y - b.Y) < epsilon; });
            points.erase(last, points.end());
            if (points.size() <= 2)
            {
                return points;
            }

            // Find the point with the lowest Y and X (O(n))
            int p0Index = 0;
            for (size_t i = 1; i < points.size(); i++)
            {
                if (points[i].Y < points[p0Index].Y - epsilon || (abs(points[i].Y - points[p0Index].Y) < epsilon && points[i].X < points[p0Index].X))
                {
                    p0Index = i;
                }
            }
            Swap(points, 0, p0Index);

            Point p0 = points[0];

            // We dont need p0 in the sorting
            // Create candidates vector from index 1 to end
            vector<Point> candidates(points.begin() + 1, points.end());

            // O(n log n)
            sort(candidates.begin(), candidates.end(), [p0](const Point &a, const Point &b)
                 {
                double cp = (a.X - p0.X) * (b.Y - p0.Y) - (a.Y - p0.Y) * (b.X - p0.X);
                if (abs(cp) < epsilon)
                {
                    // Collinear points, sort by distance to p0
                    return DistSq(p0, a) < DistSq(p0, b);
                }
                return cp > 0; });

            // We need to remove the points that are collinear with P0, keeping only the farthest one.
            // Because we sorted by distance secondary, the last point of a "same angle" group is the farthest.
            // O(n)
            vector<Point> uniqueAnglePoints;
            uniqueAnglePoints.reserve(candidates.size()); // optimization
            uniqueAnglePoints.push_back(p0);

            if (candidates.size() > 0)
            {
                for (size_t i = 0; i < candidates.size() - 1; i++)
                {
                    if (IsCollinear(p0, candidates[i], candidates[i + 1]))
                    {
                        continue;
                    }
                    uniqueAnglePoints.push_back(candidates[i]);
                }
                uniqueAnglePoints.push_back(candidates.back());
            }

            // If after filtering we don't have enough points for a polygon (meaning all points were on a line)
            if (uniqueAnglePoints.size() < 3)
            {
                return uniqueAnglePoints;
            }

            // The main Graham Scan loop
            // Using vector as a Stack
            vector<Point> stack;
            stack.reserve(uniqueAnglePoints.size()); // for optimization purposes
            stack.push_back(uniqueAnglePoints[0]);
            // We know we have at least 3 points here (triangle)
            stack.push_back(uniqueAnglePoints[1]);
            stack.push_back(uniqueAnglePoints[2]);

            // O(n), because each point is pushed exactly once and popped at most once
            for (size_t i = 3; i < uniqueAnglePoints.size(); i++)
            {
                while (stack.size() >= 2)
                {
                    // Peek at top and second-from-top
                    Point top = stack.back();
                    Point nextToTop = stack[stack.size() - 2];
                    // If right turn or collinear, pop and continue checking
                    if (Orientation(nextToTop, top, uniqueAnglePoints[i]) <= 0)
                    {
                        stack.pop_back();
                    }
                    else
                    {
                        break; // Valid left turn, stop popping
                    }
                }
                stack.push_back(uniqueAnglePoints[i]);
            }
            return stack;
        }
        // Returns > 0 for left turn, < 0 for right turn, 0 for collinear
        static double CrossProduct(Point a, Point b, Point c)
        {
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        }

        static int Orientation(Point p, Point q, Point r)
        {
            double val = CrossProduct(p, q, r);
            if (abs(val) < epsilon)
            {
                return 0; // Collinear
            }
            return (val > 0) ? 1 : -1; // 1 = counter clockwise, -1 = clockwise
        }

        static bool IsCollinear(Point p, Point q, Point r)
        {
            return abs(CrossProduct(p, q, r)) < epsilon;
        }

        static double DistSq(Point p1, Point p2)
        {
            return (p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y);
        }

        static void Swap(vector<Point> &list, int i, int j)
        {
            swap(list[i], list[j]);
        }

    public:
        static vector<Point> LoadPointsFromCSV(const string &filename)
        {
            vector<Point> points;
            ifstream file(filename);
            if (!file.is_open())
            {
                cerr << "No such file : " << filename << endl;
                return points;
            }
            string line;
            while (getline(file, line))
            {
                stringstream ss(line);
                string value;
                double x, y;
                if (!getline(ss, value, ','))
                    continue; // skip empty lines
                try
                {
                    x = stod(value);
                }
                catch (...)
                {
                    continue;
                }
                if (!getline(ss, value, ','))
                    continue;
                try
                {
                    y = stod(value);
                }
                catch (...)
                {
                    continue;
                }
                points.emplace_back(x, y);
            }
            file.close();
            return points;
        }
        static void SaveToCSV(const vector<Point> &points, const string &filename)
        {
            ofstream file(filename);
            if (!file.is_open())
            {
                cerr << "Error: Could not open file " << filename << " for writing.\n";
                return;
            }
            // Iterate in reverse for clockwise
            for (auto it = points.rbegin(); it != points.rend(); ++it)
            {
                file << it->X << "," << it->Y << "\n";
            }
            file.close();
        }
        // Points on circle (worst case for jarvis, good for graham)
        static void GenerateCircle(int n, double radius, const string &filename)
        {
            vector<Point> points;
            // Use random theta to prevent pre-sorted order
            // Initialize the Engine
            // We seed it with 42 so results are reproducible.
            mt19937 rng(42);
            // Initialize the Distribution
            // We want doubles between 10.0 and 20.0
            uniform_real_distribution<double> dist(0, 2 * M_PI);

            for (int i = 0; i < n; i++)
            {
                double theta = dist(rng);
                points.emplace_back(radius * cos(theta), radius * sin(theta));
            }
            SaveToCSV(points, filename);
        }

        // Fixed triangle with interior points (best case for jarvis)
        static void GenerateTriangle(int n, const string &filename)
        {
            vector<Point> points;
            // Use random theta to prevent pre-sorted order
            // Initialize the Engine
            // We seed it with 42 so results are reproducible.
            mt19937 rng(42);
            // Initialize the Distribution
            // We want doubles between 10.0 and 20.0
            uniform_real_distribution<double> dist(0.1, 0.9);

            // 1. Add the 3 Hull Points
            points.emplace_back(0, 0);
            points.emplace_back(1000, 0);
            points.emplace_back(500, 1000);

            // 2. Add Interior Noise (using barycentric coords to stay inside)
            // Or simpler: generate inside bounding box and check if inside triangle
            // Simple Hack: Generate in small box strictly inside
            uniform_real_distribution<double> innerDist(400, 600);
            for (int i = 0; i < n - 3; i++)
            {
                points.emplace_back(innerDist(rng), innerDist(rng));
            }
            SaveToCSV(points, filename);
        }

        // Uniform random (usually graham should win this)
        static void GenerateUniform(int n, double maxRange, const string &filename)
        {
            vector<Point> points;
            // Use random theta to prevent pre-sorted order
            // Initialize the Engine
            // We seed it with 42 so results are reproducible.
            mt19937 rng(42);
            // Initialize the Distribution
            // We want doubles between 10.0 and 20.0
            uniform_real_distribution<double> dist(0, maxRange);

            for (int i = 0; i < n; i++)
            {
                points.emplace_back(dist(rng), dist(rng));
            }
            SaveToCSV(points, filename);
        }
        // Normal test (they should technically be tied)
        static void GenerateNormal(int n, double mean, double stdDev, const string &filename)
        {
            vector<Point> points;
            // Use random theta to prevent pre-sorted order
            // Initialize the Engine
            // We seed it with 42 so results are reproducible.
            mt19937 rng(42);
            // Initialize the Distribution
            // We want doubles between 10.0 and 20.0
            // This creates a Bell Curve distribution
            normal_distribution<double> dist(mean, stdDev);

            for (int i = 0; i < n; i++)
            {
                points.emplace_back(dist(rng), dist(rng));
            }
            SaveToCSV(points, filename);
        }
        // This is just to make multiple loop runs for performance testing
        static void MakeRun(bool print, bool save)
        {
            vector<INFOGA_Bonus_CPP::Point> filePoints = INFOGA_Bonus_CPP::GrahamScan::LoadPointsFromCSV("circle.csv");
            vector<Point> hull = GetConvexHull(filePoints);
            if (print)
            {
                // Iterate in reverse for clockwise
                for (auto it = hull.rbegin(); it != hull.rend(); ++it)
                {
                    cout << "{" << it->X << " " << it->Y << "}" << endl;
                }
            }
            if (save)
            {
                GrahamScan::SaveToCSV(hull, "solution.csv");
            }
        }
    };
}

int main()
{
    //INFOGA_Bonus_CPP::GrahamScan::GenerateUniform(10000, 10000.0, "uniform.csv");
    //INFOGA_Bonus_CPP::GrahamScan::GenerateCircle(10000, 5000.0, "circle.csv");
    //INFOGA_Bonus_CPP::GrahamScan::GenerateTriangle(10000, "triangle.csv");
    //INFOGA_Bonus_CPP::GrahamScan::GenerateNormal(10000, 5000.0, 1500.0, "normal.csv");
    auto start = chrono::high_resolution_clock::now();
    int numberOfRuns = 1;
    for (int i = 0; i < numberOfRuns; i++)
    {
        INFOGA_Bonus_CPP::GrahamScan::MakeRun(false, false);
    }
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double, milli> elapsed = end - start;
    cout << "Convex Hull computed in " << elapsed.count() / numberOfRuns << " ms" << endl;
}