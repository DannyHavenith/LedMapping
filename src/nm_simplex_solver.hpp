//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//

#if !defined( NM_SIMPLEX_SOLVER_HPP_)
#define NM_SIMPLEX_SOLVER_HPP_
#include <utility> // for std::pair
#include <functional>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/array.hpp>

namespace Solvers
{
static const double alpha = 1;
static const double beta = 0.5;
static const double gamma = 2;
static const double delta = 0.5;

/// implementation of the Nelder-Mead simplex solver
template<int dimension, typename CostFunction = std::function<
        double(const boost::numeric::ublas::c_vector<double, dimension> &)> >
class NmSimplexSolver
{
public:

    /// a point in n-dimensional space
    typedef boost::numeric::ublas::c_vector<double, dimension> Point;

    /// a combination of a point in n-dimensional space and the corresponding value f(p)
    struct SimplexPoint
    {
        bool operator<(const SimplexPoint &rhs) const
        {
            return value < rhs.value;
        }

        bool operator<=(const SimplexPoint &rhs) const
        {
            return value <= rhs.value;
        }

        friend std::ostream &operator<<(std::ostream &output,
                const SimplexPoint &p)
        {
            output << '[' << p.value << ',' << p.position << "]";
            return output;
        }

        SimplexPoint(const Point &position, double value) :
                position(position), value(value)
        {
        }

        SimplexPoint() :
                position(0 * Point()), value(0.0)
        {
        }
        ;

        Point position;
        double value;
    };

    /// a simplex is a set of n + 1 points in n-dimensional space.
    /// in this particular case the set consists of both points and associated values.
    typedef boost::array<SimplexPoint, dimension + 1> Simplex;

    NmSimplexSolver(CostFunction f, double step, double epsilon, bool doReport =
            false) :
            f(f), step(step), epsilon(epsilon), lastIterationCount(0), lastCostValue(
                    0.0), doReport(doReport)
    {
    }

    Point FindMinimun(Point startingPoint, unsigned int maxIterations = 1000)
    {
        Simplex simplex = StartingSimplex(startingPoint);

        // indices to points and values. These indices have meaning and can be constant
        // because the points will have been sorted.
        const unsigned int best = 0, secondWorst = dimension - 1, worst =
                dimension;

        unsigned int iterationCount = 1;
        do
        {

            // find the centroid of all but the worst points in the simplex and reflect the worst
            // point in that centroid.
            Point centroid = FindCentroid(simplex);
            SimplexPoint reflected = PointAndValue(
                    centroid + alpha * (centroid - simplex[worst].position));

            bool doReplace = true; // true-> replace worst point, false -> shrink simplex
            SimplexPoint replacement;

            if (simplex[best] <= reflected && reflected < simplex[secondWorst])
            {
                replacement = reflected;
                Report('r', simplex); // reflect
            }
            else if (reflected < simplex[best])
            {
                SimplexPoint expanded = PointAndValue(
                        centroid
                                + gamma * (centroid - simplex[worst].position));
                if (expanded < reflected)
                {
                    replacement = expanded;
                    Report('e', simplex); // expand
                }
                else
                {
                    replacement = reflected;
                    Report('r', simplex);
                }
            }
            else // reflected >= simplex[secondWorst]
            {
                if (reflected < simplex[worst])
                {
                    SimplexPoint contracted = PointAndValue(
                            centroid + beta * (reflected.position - centroid));
                    if (contracted <= simplex[worst])
                    {
                        replacement = contracted;
                        Report('c', simplex); // contract (outer)
                    }
                    else
                    {
                        doReplace = false; // shrink
                    }
                }
                else
                {
                    SimplexPoint contracted = PointAndValue(
                            centroid
                                    + beta
                                            * (simplex[worst].position
                                                    - centroid));
                    // notice the '<' instead of '<='
                    if (contracted < simplex[worst])
                    {
                        replacement = contracted;
                        Report('i', simplex); // 'inner' contract
                    }
                    else
                    {
                        doReplace = false; // shrink
                    }
                }
            }

            if (doReplace)
            {
                simplex[worst] = replacement;
                // place the last point of the simplex at the right location in the sorted simplex
                std::inplace_merge(simplex.begin(), simplex.end() - 1,
                        simplex.end());
            }
            else
            {
                // as delta < 1 the grow function will actually shrink the simplex
                Report('s', simplex); // 'shrink'
                Grow(simplex, delta);
                Sort(simplex);
            }

        } while (++iterationCount <= maxIterations
                && (simplex[worst].value - simplex[best].value > epsilon));

        lastIterationCount = iterationCount - 1;
        lastCostValue = simplex[best].value;
        return simplex[best].position;

    }

    unsigned int GetLastIterationCount() const
    {
        return lastIterationCount;
    }

    double GetLastCostValue() const
    {
        return lastCostValue;
    }

    /// for debugging purposes, return the epsilon values for the last 2 iterations
    const boost::numeric::ublas::c_vector<double, 2> GetEpsilons() const
    {
        return epsilons;
    }

private:
    /// for debugging purposes, report on the specific iteratrion step that was taken.
    /// the step should be 'r'eflect, 'e'xpand, 'c'ontract, 'i'nner contract, 's'hrink.
    void Report(char what, const Simplex &simplex)
    {
        if (doReport)
        {
            std::cout << what << '\t'
                    << simplex.back().value - simplex.front().value << '\t'
                    << simplex.front().value << '\n';
        }
    }

    /// given a point position, return a simplexPoint that stores this position and the
    /// corresponding value
    SimplexPoint PointAndValue(const Point &p) const
    {
        return SimplexPoint(p, f(p));
    }

    /// find the gravitational center of all but the last point in the simplex.
    static Point FindCentroid(const Simplex &sortedSimplex)
    {
        Point centroid = boost::numeric::ublas::zero_vector<double>(dimension);
        for (auto i = sortedSimplex.begin(); i + 1 < sortedSimplex.end(); ++i)
        {
            centroid += i->position;
        }
        centroid /= dimension;

        return centroid;
    }

    /// Grow (factor > 1)  or shrink (factor < 1) all points in a simplex towards the first point.
    void Grow(Simplex &simplex, double factor) const
    {
        Point firstPoint = simplex[0].position;
        for (auto i = simplex.begin() + 1; i < simplex.end(); ++i)
        {
            *i = PointAndValue(
                    firstPoint + factor * (i->position - firstPoint));
        }
    }

    /// sort the simplex points on value (not on point position in space...)
    static void Sort(Simplex &simplex)
    {
        std::sort(simplex.begin(), simplex.end());
    }

    /// Create a sorted starting simplex given a starting point.
    /// The starting simplex consists of the starting point and all points at right angles, at distance 'step'
    Simplex StartingSimplex(const Point &startingPoint) const
    {
        using boost::numeric::ublas::unit_vector;

        Simplex simplex;

        simplex[0] = PointAndValue(startingPoint);
        for (unsigned int i = 1; i < simplex.size(); ++i)
        {
            simplex[i] = PointAndValue(
                    startingPoint
                            + step * unit_vector<double>(dimension, i - 1));
        }

        Sort(simplex);
        return simplex;
    }

    CostFunction f;
    const double step;
    const double epsilon;
    unsigned int lastIterationCount;
    double lastCostValue;
    boost::numeric::ublas::c_vector<double, 2> epsilons;
    bool doReport;
};
}
#endif //NM_SIMPLEX_SOLVER_HPP_
