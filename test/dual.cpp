#include "test_macros.hpp"
#include <matrix/math.hpp>
#include <iostream>

using namespace matrix;

template <typename Scalar, size_t N>
bool isEqualAll(Dual<Scalar, N> a, Dual<Scalar, N> b)
{
    return isEqualF(a.value, b.value) && a.derivative == b.derivative;
}

template <typename T>
T testFunction(const Vector<T, 3>& point) {
    // function is f(x,y,z) = x^2 + 2xy + 3y^2 + z
    return point(0)*point(0)
           + 2.f * point(0) * point(1)
           + 3.f * point(1) * point(1)
           + point(2);
}

template <typename Scalar>
Vector<Scalar, 3> positionError(const Vector<Scalar, 3>& positionState,
                                const Vector<Scalar, 3>& velocityStateBody,
                                const Quaternion<Scalar>& bodyOrientation,
                                const Vector<Scalar, 3>& positionMeasurement,
                                Scalar dt
                               )
{
    return positionMeasurement - (positionState +  bodyOrientation.conjugate(velocityStateBody) * dt);
}

int main()
{
    const Dual<float, 1> a(3,0);
    const Dual<float, 1> b(6,0);

    {
        TEST(isEqualF(a.value, 3.f));
        TEST(isEqualF(a.derivative(0), 1.f));
    }

    {
        // addition
        Dual<float, 1> c = a + b;
        TEST(isEqualF(c.value, 9.f));
        TEST(isEqualF(c.derivative(0), 2.f));

        Dual<float, 1> d = +a;
        TEST(isEqualAll(d, a));
        d += b;
        TEST(isEqualAll(d, c));

        Dual<float, 1> e = a;
        e += b.value;
        TEST(isEqualF(e.value, c.value));
        TEST(isEqual(e.derivative, a.derivative));

        Dual<float, 1> f = b.value + a;
        TEST(isEqualAll(f, e));
    }

    {
        // subtraction
        Dual<float, 1> c = b - a;
        TEST(isEqualF(c.value, 3.f));
        TEST(isEqualF(c.derivative(0), 0.f));

        Dual<float, 1> d = b;
        TEST(isEqualAll(d, b));
        d -= a;
        TEST(isEqualAll(d, c));

        Dual<float, 1> e = b;
        e -= a.value;
        TEST(isEqualF(e.value, c.value));
        TEST(isEqual(e.derivative, b.derivative));

        Dual<float, 1> f = a.value - b;
        TEST(isEqualAll(f, -e));
    }

    {
        // multiplication
        Dual<float, 1> c = a*b;
        TEST(isEqualF(c.value, 18.f));
        TEST(isEqualF(c.derivative(0), 9.f));

        Dual<float, 1> d = a;
        TEST(isEqualAll(d, a));
        d *= b;
        TEST(isEqualAll(d, c));

        Dual<float, 1> e = a;
        e *= b.value;
        TEST(isEqualF(e.value, c.value));
        TEST(isEqual(e.derivative, a.derivative * b.value));

        Dual<float, 1> f = b.value * a;
        TEST(isEqualAll(f, e));
    }

    {
        // division
        Dual<float, 1> c = b/a;
        TEST(isEqualF(c.value, 2.f));
        TEST(isEqualF(c.derivative(0), -1.f/3.f));

        Dual<float, 1> d = b;
        TEST(isEqualAll(d, b));
        d /= a;
        TEST(isEqualAll(d, c));

        Dual<float, 1> e = b;
        e /= a.value;
        TEST(isEqualF(e.value, c.value));
        TEST(isEqual(e.derivative, b.derivative / a.value));

        Dual<float, 1> f = a.value / b;
        TEST(isEqualAll(f, 1.f/e));
    }

    {
        Dual<float, 1> blank;
        TEST(isEqualF(blank.value, 0.f));
        TEST(isEqualF(blank.derivative(0), 0.f));
    }

    {
        // sqrt
        TEST(isEqualF(sqrt(a).value, sqrt(a.value)));
        TEST(isEqualF(sqrt(a).derivative(0), 1.f/sqrt(12.f)));
    }

    {
        // abs
        TEST(isEqualAll(a, abs(-a)));
        TEST(!isEqualAll(-a, abs(a)));
        TEST(isEqualAll(-a, -abs(a)));
    }

    {
        // ceil
        Dual<float, 1> c(1.5,0);
        TEST(isEqualF(ceil(c).value, ceil(c.value)));
        TEST(isEqualF(ceil(c).derivative(0), 0.f));
    }

    {
        // floor
        Dual<float, 1> c(1.5,0);
        TEST(isEqualF(floor(c).value, floor(c.value)));
        TEST(isEqualF(floor(c).derivative(0), 0.f));
    }

    {
        // fmod
        TEST(isEqualF(fmod(a, 0.8f).value, fmod(a.value, 0.8f)));
        TEST(isEqual(fmod(a, 0.8f).derivative, a.derivative));
    }

    {
        // max/min
        TEST(isEqualAll(b, max(a, b)));
        TEST(isEqualAll(a, min(a, b)));
    }

    {
        // isnan
        TEST(!isnan(a));
        Dual<float, 1> c(sqrt(-1.f),0);
        TEST(isnan(c));
    }

    {
        // isfinite/isinf
        TEST(isfinite(a));
        TEST(!isinf(a));
        Dual<float, 1> c(sqrt(-1.f),0);
        TEST(!isfinite(c));
        TEST(!isinf(c));
        Dual<float, 1> d(INFINITY,0);
        TEST(!isfinite(d));
        TEST(isinf(d));
    }

    {
        // sin/cos/tan
        TEST(isEqualF(sin(a).value, sin(a.value)));
        TEST(isEqualF(sin(a).derivative(0), cos(a.value))); // sin'(x) = cos(x)

        TEST(isEqualF(cos(a).value, cos(a.value)));
        TEST(isEqualF(cos(a).derivative(0), -sin(a.value))); // cos'(x) = -sin(x)

        TEST(isEqualF(tan(a).value, tan(a.value)));
        TEST(isEqualF(tan(a).derivative(0), 1.f + tan(a.value)*tan(a.value))); // tan'(x) = 1 + tan^2(x)
    }

    {
        // asin/acos/atan
        Dual<float, 1> c(0.3f, 0);
        TEST(isEqualF(asin(c).value, asin(c.value)));
        TEST(isEqualF(asin(c).derivative(0), 1.f/sqrt(1.f - 0.3f*0.3f))); // asin'(x) = 1/sqrt(1-x^2)

        TEST(isEqualF(acos(c).value, acos(c.value)));
        TEST(isEqualF(acos(c).derivative(0), -1.f/sqrt(1.f - 0.3f*0.3f))); // acos'(x) = -1/sqrt(1-x^2)

        TEST(isEqualF(atan(c).value, atan(c.value)));
        TEST(isEqualF(atan(c).derivative(0), 1.f/(1.f + 0.3f*0.3f))); // tan'(x) = 1 + x^2
    }

    {
        // atan2
        TEST(isEqualF(atan2(a, b).value, atan2(a.value, b.value)));
        TEST(isEqualAll(atan2(a, Dual<float,1>(b.value)), atan(a/b.value))); // atan2'(y, x) = atan'(y/x)
    }

    {
        // partial derivatives
        // function is f(x,y,z) = x^2 + 2xy + 3y^2 + z, we need with respect to d/dx and d/dy at the point (0.5, -0.8, 2)

        Vector<Dual<float, 2>, 3> dualPoint;
        dualPoint(0).value = 0.5f;
        dualPoint(1).value = -0.8f;
        dualPoint(2).value = 2.f;
        dualPoint(0).derivative(0) = 1.f;
        dualPoint(1).derivative(1) = 1.f;

        Dual<float, 2> dualResult = testFunction(dualPoint);

        // compare to a numerical derivative:
        Vector<float, 3> floatPoint;
        floatPoint(0) = dualPoint(0).value;
        floatPoint(1) = dualPoint(1).value;
        floatPoint(2) = dualPoint(2).value;
        float floatResult = testFunction(floatPoint);
        float h = 0.0001f;
        Vector<float, 3> floatPoint_plusDX = floatPoint;
        floatPoint_plusDX(0) += h;
        float floatResult_plusDX = testFunction(floatPoint_plusDX);
        Vector<float, 3> floatPoint_plusDY = floatPoint;
        floatPoint_plusDY(1) += h;
        float floatResult_plusDY = testFunction(floatPoint_plusDY);

        TEST(isEqualF(dualResult.derivative(0), (floatResult_plusDX - floatResult)/h, 1e-3f));
        TEST(isEqualF(dualResult.derivative(1), (floatResult_plusDY - floatResult)/h, 1e-2f));

    }

    {
        // jacobian
        // get residual of x/y/z with partial derivatives of velocity error given x/y/z and vx/vy/vz

        Vector3f direct_error;
        SquareMatrix3f numerical_jacobian;
        {
            Vector3f positionState(5,6,7);
            Vector3f velocityState(-1,0,1);
            Quaternionf velocityOrientation(0.2f,-0.1f,0,1);
            Vector3f positionMeasurement(4.5f, 6.2f, 7.9f);
            float dt = 0.1f;

            direct_error = positionError(positionState,
                                         velocityState,
                                         velocityOrientation,
                                         positionMeasurement,
                                         dt);
            float h = 0.001f;
            for (size_t i = 0; i < 3; i++)
            {
                Vector3f h3;
                h3(i) = h;
                numerical_jacobian.col(i) = (positionError(positionState,
                                             velocityState + h3,
                                             velocityOrientation,
                                             positionMeasurement,
                                             dt)
                                             - direct_error)/h;
            }
        }
        Vector3f auto_error;
        SquareMatrix3f auto_jacobian;
        {
            using D = Dual<float, 3>;
            using Vector3f3d = Vector3<D>;
            Vector3f3d positionState(D(5), D(6), D(7));
            Vector3f3d velocityState(D(-1), D(0), D(1));
            Quaternion<D> velocityOrientation(D(0.2f),D(-0.1f),D(0),D(1));
            Vector3f3d positionMeasurement(D(4.5f), D(6.2f), D(7.9f));
            D dt(0.1f);

            // get partial derivatives of velocity state:
            velocityState(0).derivative(0) = 1;
            velocityState(1).derivative(1) = 1;
            velocityState(2).derivative(2) = 1;

            Vector3f3d error = positionError(positionState,
                                             velocityState,
                                             velocityOrientation,
                                             positionMeasurement,
                                             dt);
            auto_error = Vector3f(error(0).value, error(1).value, error(2).value);
            auto_jacobian.row(0) = error(0).derivative;
            auto_jacobian.row(1) = error(1).derivative;
            auto_jacobian.row(2) = error(2).derivative;
        }
        TEST(isEqual(direct_error, auto_error));
        TEST(isEqual(numerical_jacobian, auto_jacobian, 1e-3f));

    }
    return 0;
}
