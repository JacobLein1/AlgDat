import java.util.Date;

public class Oving2 {
    public static void main(String[] args) {
        double base = 1.002;

        System.out.println("Sjekk for grunntall 5, eksponent 11: ");
        System.out.println("Method 1, O(n)" + recOne(5, 11) + "\n");
        System.out.println("Method 2, O(log(n))" + recTwo(5, 11) + "\n");
        System.out.println("Method 3, Math.pow()" + Math.pow(5, 11) + "\n");

        int[] nValues = { 1000, 2001, 4000, 8001, 16000 };
        int[] nValues2 = { 1000, 2001, 4000, 8001, 16000, 32001, 64000 };
        System.out.println("Method 1, O(n)");
        System.out.println("-----------------");
        runTimeTest(0, nValues, base);
        System.out.println("\n");
        System.out.println("Method 2, O(log(n))");
        System.out.println("-----------------");
        runTimeTest(1, nValues2, base);
        System.out.println("\n");
        System.out.println("Method 3, Math.pow()");
        System.out.println("-----------------");
        runTimeTest(2, nValues, base);
        System.out.println("\n");
    }

    private static void runTimeTest(int methodIndex, int[] nValues, double base) {

        for (int i = 0; i <= nValues.length - 1; i += 1) {
            int expo = nValues[i];
            double time = 0;
            int rounds = 0;
            Date endTime;
            double result = 0;
            String formattedResult = "";
            Date startTime = new Date();
            do {
                switch (methodIndex) {
                    case 0:
                        result = recOne(base, nValues[i]);
                        break;
                    case 1:
                        result = recTwo(base, nValues[i]);
                        break;
                    default:
                        result = Math.pow(base, nValues[i]);
                }
                endTime = new Date();
                rounds++;
            } while (endTime.getTime() - startTime.getTime() < 1000);
            formattedResult = String.format("%.5f", result);
            time = (double) (endTime.getTime() - startTime.getTime()) / rounds;
            System.out.println("\n" + base + "^" + nValues[i] + " = " + formattedResult + " Time: " + time + "ms");
        }
    }

    private static double recOne(double base, int n) {
        double sum = 0;
        if (n == 1) {
            return base;
        } else {
            sum = base * recOne(base, (n - 1));
            return sum;
        }
    }

    private static double recTwo(double base, int n) {
        double sum = 0;
        if (n == 1) {
            return base;
        }
        if ((n & 1) == 1) {
            sum = base * recTwo(base * base, (n - 1) / 2);
        } else {
            sum = recTwo(base * base, (n / 2));
        }
        return sum;
    }
}
