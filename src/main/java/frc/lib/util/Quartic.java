package frc.lib.util;

public class Quartic {
    public static double eps = 1e-12;

    public static int solveP3(double[] x, double a, double b, double c) {
        double a2 = a*a;
        double q = (a2 - 3*b)/9;
        double r  = (a*(2*a2-9*b) + 27*c)/54;
    	double r2 = r*r;
	    double q3 = q*q*q;
        double A, B;
        if (r2<q3) {
            double t=r/Math.sqrt(q3);
    		if( t<-1) t=-1;
    		if( t> 1) t= 1;
    		t=Math.acos(t);
    		a/=3; q=-2*Math.sqrt(q);
    		x[0]=q*Math.cos(t/3)-a;
    		x[1]=q*Math.cos((t+2*Math.PI)/3)-a;
    		x[2]=q*Math.cos((t-2*Math.PI)/3)-a;
    		return 3;
        } else {
            A =-Math.pow(Math.abs(r)+Math.sqrt(r2-q3),1./3);
    		if( r<0 ) A=-A;
    		B = (0==A ? 0 : q/A);

            a/=3;
            x[0] =(A+B)-a;
            x[1] =-0.5*(A+B)-a;
            x[2] = 0.5*Math.sqrt(3.)*(A-B);
            if(Math.abs(x[2])<eps) { x[2]=x[1]; return 2; }
            
            return 1;
        }
    }

    public static ComplexNumber[] solveQuartic(double c0, double a, double b, double c, double d) {
        a = a / c0;
        b = b / c0;
        c = c / c0;
        d = d / c0;

        double a3 = -b;
        double b3 =  a*c -4.*d;
        double c3 = -a*a*d - c*c + 4.*b*d;

        double[] x3 = new double[3];
        int iZeroes = solveP3(x3, a3, b3, c3);

        double q1, q2, p1, p2, D, sqD, y;

        y = x3[0];

        if (iZeroes != 1) {
            if(Math.abs(x3[1]) > Math.abs(y)) y = x3[1];
            if(Math.abs(x3[2]) > Math.abs(y)) y = x3[2];
        }

        D = y*y - 4*d;
        if (Math.abs(D) < eps) {
            q1 = q2 = y * 0.5;
            D = a*a - 4*(b-y);
            if (Math.abs(D) < eps) {
                p1 = p2 = a * 0.5;
            } else {
                sqD = Math.sqrt(D);
                p1 = (a + sqD) * 0.5;
                p2 = (a - sqD) * 0.5;
            }
        } else {
            sqD = Math.sqrt(D);
            q1 = (y + sqD) * 0.5;
            q2 = (y - sqD) * 0.5;
            p1 = (a*q1-c)/(q1-q2);
            p2 = (c-a*q2)/(q1-q2);
        }
    
        ComplexNumber[] retval = new ComplexNumber[4];

        D = p1*p1 - 4*q1;
        if (D < 0.0) {
            retval[0].real( -p1 * 0.5 );
            retval[0].imag( Math.sqrt(-D) * 0.5 );
            retval[1] = retval[0].conjugate();
        } else {
            sqD = Math.sqrt(D);
            retval[0].real( (-p1 + sqD) * 0.5 );
            retval[1].real( (-p1 - sqD) * 0.5 );
        }

        D = p2*p2 - 4*q2;
        if (D < 0.0) {
            retval[2].real( -p2 * 0.5 );
            retval[2].imag( Math.sqrt(-D) * 0.5 );
            retval[3] = retval[2].conjugate();
        } else {
            sqD = Math.sqrt(D);
            retval[2].real( (-p2 + sqD) * 0.5 );
            retval[3].real( (-p2 - sqD) * 0.5 );
        }

        return retval;
    }
}
