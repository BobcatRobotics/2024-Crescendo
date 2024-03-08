// package frc.lib.util;

// public class Quartic {
//     public static double eps = 1e-12;

//     public static int solveP3(double[] x, double a, double b, double c) {
//         double a2 = a*a;
//         double q = (a2 - 3*b)/9;
//         double r  = (a*(2*a2-9*b) + 27*c)/54;
//     	double r2 = r*r;
// 	    double q3 = q*q*q;
//         double A, B;
//         if (r2<q3) {
//             double t=r/Math.sqrt(q3);
//     		if( t<-1) t=-1;
//     		if( t> 1) t= 1;
//     		t=Math.acos(t);
//     		a/=3; q=-2*Math.sqrt(q);
//     		x[0]=q*Math.cos(t/3)-a;
//     		x[1]=q*Math.cos((t+2*Math.PI)/3)-a;
//     		x[2]=q*Math.cos((t-2*Math.PI)/3)-a;
//     		return 3;
//         } else {
//             A =-Math.pow(Math.abs(r)+Math.sqrt(r2-q3),1./3);
//     		if( r<0 ) A=-A;
//     		B = (0==A ? 0 : q/A);

//             a/=3;
//             x[0] =(A+B)-a;
//             x[1] =-0.5*(A+B)-a;
//             x[2] = 0.5*Math.sqrt(3.)*(A-B);
//             if(Math.abs(x[2])<eps) { x[2]=x[1]; return 2; }
            
//             return 1;
//         }
//     }

//     public static ComplexNumber[] solveQuartic(double c0, double a, double b, double c, double d) {
//         a = a / c0;
//         b = b / c0;
//         c = c / c0;
//         d = d / c0;

//         double a3 = -b;
//         double b3 =  a*c -4.*d;
//         double c3 = -a*a*d - c*c + 4.*b*d;

//         double[] x3 = new double[3];
//         int iZeroes = solveP3(x3, a3, b3, c3);

//         double q1, q2, p1, p2, D, sqD, y;

//         y = x3[0];

//         if (iZeroes != 1) {
//             if(Math.abs(x3[1]) > Math.abs(y)) y = x3[1];
//             if(Math.abs(x3[2]) > Math.abs(y)) y = x3[2];
//         }

//         D = y*y - 4*d;
//         if (Math.abs(D) < eps) {
//             q1 = q2 = y * 0.5;
//             D = a*a - 4*(b-y);
//             if (Math.abs(D) < eps) {
//                 p1 = p2 = a * 0.5;
//             } else {
//                 sqD = Math.sqrt(D);
//                 p1 = (a + sqD) * 0.5;
//                 p2 = (a - sqD) * 0.5;
//             }
//         } else {
//             sqD = Math.sqrt(D);
//             q1 = (y + sqD) * 0.5;
//             q2 = (y - sqD) * 0.5;
//             p1 = (a*q1-c)/(q1-q2);
//             p2 = (c-a*q2)/(q1-q2);
//         }
    
//         ComplexNumber[] retval = new ComplexNumber[4];
//         for (int i = 0; i < retval.length; i++) {
//             retval[i] = new ComplexNumber();
//         }

//         D = p1*p1 - 4*q1;
//         if (D < 0.0) {
//             retval[0].real( -p1 * 0.5 );
//             retval[0].imag( Math.sqrt(-D) * 0.5 );
//             retval[1] = retval[0].conjugate();
//         } else {
//             sqD = Math.sqrt(D);
//             retval[0].real( (-p1 + sqD) * 0.5 );
//             retval[1].real( (-p1 - sqD) * 0.5 );
//         }

//         D = p2*p2 - 4*q2;
//         if (D < 0.0) {
//             retval[2].real( -p2 * 0.5 );
//             retval[2].imag( Math.sqrt(-D) * 0.5 );
//             retval[3] = retval[2].conjugate();
//         } else {
//             sqD = Math.sqrt(D);
//             retval[2].real( (-p2 + sqD) * 0.5 );
//             retval[3].real( (-p2 - sqD) * 0.5 );
//         }

//         return retval;
//     }
// }

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