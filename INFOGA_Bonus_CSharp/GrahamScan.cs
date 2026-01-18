namespace INFOGA_Bonus_CSharp
{
    public class GrahamScan
    {
        // High precision epsilon to handle "almost" collinear cases
        private const double epsilon = 1e-9;

        public struct Point(double x, double y)
        {
            public double X = x, Y = y;
        }
        // The algorithm is clockwise
        public static Stack<Point> GetConvexHull(List<Point> inputPoints)
        {
            // Remove duplicates to prevent division zero errors
            var points = inputPoints.Distinct().ToList();
            if (points.Count <= 2)
            {
                return new Stack<Point>(points);
            }
            // Find the point with the lowest Y and X (O(n))
            int p0Index = 0;
            for (int i = 1; i < points.Count; i++)
            {
                if (points[i].Y < points[p0Index].Y - epsilon || (Math.Abs(points[i].Y - points[p0Index].Y) < epsilon && points[i].X < points[p0Index].X))
                {
                    p0Index = i;
                }
            }
            Swap(points, 0, p0Index);

            Point p0 = points[0];
            // We dont need p0 in the sorting
            var candidates = points.GetRange(1, points.Count - 1);
            //O(n log n)
            candidates.Sort((a, b) =>
            {
                // We use atan2 to get the angle from P0 to each point, atan2 handles degenerate cases better than slope
                //O(1)
                double angleA = Math.Atan2(a.Y - p0.Y, a.X - p0.X);
                double angleB = Math.Atan2(b.Y - p0.Y, b.X - p0.X);
                if (Math.Abs(angleA - angleB) < epsilon)
                {
                    // We choose to keep the farthest point in case of tie,
                    // because in CH the furthest point defines the hull better.
                    double distA = DistSq(p0, a);
                    double distB = DistSq(p0, b);
                    return distA.CompareTo(distB);
                }
                return angleA.CompareTo(angleB);
            });

            // We need to remove the points that are collinear with P0, keeping only the farthest one.
            // Because we sorted by distance secondary, the last point of a "same angle" group is the farthest.
            //O(n)
            List<Point> uniqueAnglePoints =
            [
                p0,
            ];
            if (candidates.Count > 0)
            {
                for (int i = 0; i < candidates.Count - 1; i++)
                {
                    if (IsCollinear(p0, candidates[i], candidates[i + 1]))
                    {
                        continue;
                    }
                    uniqueAnglePoints.Add(candidates[i]);
                }
                uniqueAnglePoints.Add(candidates[^1]);
            }
            // If after filtering we don't have enough points for a polygon (meaning all points were on a line)
            if (uniqueAnglePoints.Count < 3)
            {
                return new Stack<Point>(uniqueAnglePoints);
            }

            // The main Graham Scan loop
            Stack<Point> stack = new();
            stack.Push(uniqueAnglePoints[0]);
            // We know we have at least 3 points here (triangle)
            stack.Push(uniqueAnglePoints[1]);
            stack.Push(uniqueAnglePoints[2]);
            //O(n), because each point is pushed exactly once and popped at most once
            for (int i = 3; i < uniqueAnglePoints.Count; i++)
            {
                Point top = stack.Pop();
                // We only want Negative values (Right Turns).
                while (stack.Count > 0 && Orientation(stack.Peek(), top, uniqueAnglePoints[i]) <= 0)
                {
                    top = stack.Pop();
                }

                stack.Push(top);
                stack.Push(uniqueAnglePoints[i]);
            }
            return stack;
        }

        // Returns > 0 for left turn, < 0 for right turn, 0 for collinear
        private static double CrossProduct(Point a, Point b, Point c)
        {
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        }
        private static int Orientation(Point p, Point q, Point r)
        {
            double val = CrossProduct(p, q, r);
            if (Math.Abs(val) < epsilon)
            {
                return 0; // Collinear
            }
            return (val > 0) ? 1 : -1; // 1 = counter clockwise, -1 = clockwise
        }
        private static bool IsCollinear(Point p, Point q, Point r)
        {
            return Math.Abs(CrossProduct(p, q, r)) < epsilon;
        }
        private static double DistSq(Point p1, Point p2)
        {
            return (p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y);
        }
        private static void Swap(List<Point> list, int i, int j)
        {
            (list[j], list[i]) = (list[i], list[j]);
        }
        public static void Main()
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            int numberOfRuns = 1000;
            for (int i = 0 ; i < numberOfRuns; i++)
            {
                MakeRun(false);
            }
            sw.Stop();
            Console.WriteLine($"Convex Hull computed in {(double)sw.ElapsedMilliseconds / (double)numberOfRuns} ms");
        }
        // This is just to make multiple loop runs for performance testing
        // (1 run is too fast to measure, but 1000 runs of the same thing can be measurable by the C# stopwatch)
        public static void MakeRun(bool print)
        {
            var points = new List<Point>
            {
                new(0, 0),
                new(1, 2),
                new(2, 3),
                new(5, 5),
                new(0, 5),
                new(5, 0),
            };
            var hull = GetConvexHull(points);
            if (print)
            {
                foreach (var p in hull)
                {
                    Console.WriteLine("{" + p.X + " " + p.Y + "}");
                }
            }
        }
    }
}