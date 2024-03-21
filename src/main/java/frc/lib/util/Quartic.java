package frc.lib.util;

public class Quartic {
    private static boolean IsZero(double d) {
        double eps = 1e-9;
        return d > -eps && d < eps;
    }

    private static double getCubicRoot(double value) {   
        if (value > 0.0) {
            return Math.pow(value, 1.0 / 3.0);
        } else if (value < 0) {
            return -Math.pow(-value, 1.0 / 3.0);
        } else {
            return 0.0;
        }
    }

    // Solve quadratic equation: c0*x^2 + c1*x + c2. 
    // Returns number of solutions.
    private static double[] solveQuadric(double c0, double c1, double c2) {
        double[] ret_val = new double[3];

        double p, q, D;

        /* normal form: x^2 + px + q = 0 */
        p = c1 / (2 * c0);
        q = c2 / c0;

        D = p * p - q;

        if (IsZero(D)) {
	        ret_val[0] = -p;
            ret_val[2] = 1;
	        return ret_val;
        }
        else if (D < 0) {
	        return new double[3];
        }
        else /* if (D > 0) */ {
	        double sqrt_D = Math.sqrt(D);

	        ret_val[0] =   sqrt_D - p;
	        ret_val[1] = -sqrt_D - p;
            ret_val[2] = 2;
	        return ret_val;
        }
    }

    private static double[] solveCubic(double c0, double c1, double c2, double c3) {
        double[] ret_val = new double[4];

        int     num;
        double  sub;
        double  A, B, C;
        double  sq_A, p, q;
        double  cb_p, D;

        /* normal form: x^3 + Ax^2 + Bx + C = 0 */
        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;

        /*  substitute x = y - A/3 to eliminate quadric term:  x^3 +px + q = 0 */
        sq_A = A * A;
        p = 1.0/3 * (- 1.0/3 * sq_A + B);
        q = 1.0/2 * (2.0/27 * A * sq_A - 1.0/3 * A * B + C);

        /* use Cardano's formula */
        cb_p = p * p * p;
        D = q * q + cb_p;

        if (IsZero(D)) {
	        if (IsZero(q)) /* one triple solution */ {
	            ret_val[0] = 0;
	            num = 1;
	        }
	        else /* one single and one double solution */ {
	            double u = getCubicRoot(-q);
	            ret_val[0] = 2 * u;
	            ret_val[1] = - u;
	            num = 2;
	        }
        } else if (D < 0) /* Casus irreducibilis: three real solutions */ {
	        double phi = 1.0/3 * Math.acos(-q / Math.sqrt(-cb_p));
	        double t = 2 * Math.sqrt(-p);

	        ret_val[0] =   t * Math.cos(phi);
	        ret_val[1] = - t * Math.cos(phi + Math.PI / 3);
	        ret_val[2] = - t * Math.cos(phi - Math.PI / 3);
	        num = 3;
        } else /* one real solution */ {
	        double sqrt_D = Math.sqrt(D);
            double u = getCubicRoot(sqrt_D - q);
            double v = -getCubicRoot(sqrt_D + q);

	        ret_val[0] = u + v;
	        num = 1;
        }

        /* resubstitute */
        sub = 1.0/3 * A;

        if (num > 0)    ret_val[0] -= sub;
        if (num > 1)    ret_val[1] -= sub;
        if (num > 2)    ret_val[2] -= sub;

        ret_val[3] = num;
        return ret_val;
    }

    public static double[] solveQuartic(double c0, double c1, double c2, double c3, double c4) {
        double[] ret_val = new double[5];

        double[] coeffs = new double[4];
        double  z, u, v, sub;
        double  A, B, C, D;
        double  sq_A, p, q, r;
        int     num;

        /* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */
        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;
        D = c4 / c0;

        /*  substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0 */
        sq_A = A * A;
        p = - 3.0/8 * sq_A + B;
        q = 1.0/8 * sq_A * A - 1.0/2 * A * B + C;
        r = - 3.0/256*sq_A*sq_A + 1.0/16*sq_A*B - 1.0/4*A*C + D;

        if (IsZero(r)) {
	        /* no absolute term: y(y^3 + py + q) = 0 */

	        coeffs[ 3 ] = q;
	        coeffs[ 2 ] = p;
	        coeffs[ 1 ] = 0;
	        coeffs[ 0 ] = 1;

            double[] cubic_sol = solveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
            ret_val[0] = cubic_sol[0];
            ret_val[1] = cubic_sol[1];
            ret_val[2] = cubic_sol[2];
	        num = (int)cubic_sol[3];
        } else {
	        /* solve the resolvent cubic ... */
	        coeffs[ 3 ] = 1.0/2 * r * p - 1.0/8 * q * q;
	        coeffs[ 2 ] = - r;
	        coeffs[ 1 ] = - 1.0/2 * p;
	        coeffs[ 0 ] = 1;

            double[] cubic_sol = solveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
            ret_val[0] = cubic_sol[0];
            ret_val[1] = cubic_sol[1];
            ret_val[2] = cubic_sol[2];

	        /* ... and take the one real solution ... */
	        z = ret_val[0];

	        /* ... to build two quadric equations */
	        u = z * z - r;
	        v = 2 * z - p;

	        if (IsZero(u))
	            u = 0;
	        else if (u > 0)
	            u = Math.sqrt(u);
	        else
	            return new double[5];

	        if (IsZero(v))
	            v = 0;
	        else if (v > 0)
	            v = Math.sqrt(v);
	        else
	            return new double[5];

	        coeffs[ 2 ] = z - u;
	        coeffs[ 1 ] = q < 0 ? -v : v;
	        coeffs[ 0 ] = 1;

	        num = (int)solveQuadric(coeffs[0], coeffs[1], coeffs[2])[2];

	        coeffs[ 2 ]= z + u;
	        coeffs[ 1 ] = q < 0 ? v : -v;
	        coeffs[ 0 ] = 1;

            if (num == 0) num += (int)solveQuadric(coeffs[0], coeffs[1], coeffs[2])[2];
            else if (num == 1) num += (int)solveQuadric(coeffs[0], coeffs[1], coeffs[2])[2];
            else if (num == 2) num += (int)solveQuadric(coeffs[0], coeffs[1], coeffs[2])[2];
        }

        /* resubstitute */
        sub = 1.0/4 * A;

        if (num > 0)    ret_val[0] -= sub;
        if (num > 1)    ret_val[1] -= sub;
        if (num > 2)    ret_val[2] -= sub;
        if (num > 3)    ret_val[3] -= sub;
        ret_val[4] = num;

        return ret_val;
    }
}