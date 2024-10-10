
import java.util.Random;
import java.util.ArrayList;

public class Main {
    public static void main(String[] args) {
        Table table = new Table(new int[2]);

        int[] testColumnAmount = { 1000, 1500, 2000, 3000, 4000, 6000, 8000, 10000 };
        for (int i = 0; i < testColumnAmount.length; i++) {
            Table newTable = table.generateTable(testColumnAmount[i]);

            long startTime = System.currentTimeMillis();
            int[] result = SearchAlg.findBestProfit(newTable.table);
            long elapsedTime = System.currentTimeMillis() - startTime;

            System.out.println("\n");
            System.out.println("Columns: " + testColumnAmount[i] + " Time: " + elapsedTime + "ms ");
            System.out.println("Best profit: " + result[0] + " from day: " + result[1] + " to day: " + result[2]);
        }
        int[] testArray = { -1, 3, -9, 2, 2, -1, 2, -1, -5 };
        int[] exampleResult = SearchAlg.findBestProfit(testArray);
        System.out.println("\n");
        System.out.println("Example array: ");
        System.out.println(
                "Best profit: " + exampleResult[0] + " from day: " + exampleResult[1] + " to day: " + exampleResult[2]);

    }

    public static class SearchAlg {

        // Returns array where first element is the best profit, second element is the
        // buy-date and third element is the sell-date
        public static int[] findBestProfit(int[] pricePerDayList) {
            int startDate = 0;
            int endDate = 0;
            int maxProfit = Integer.MIN_VALUE;

            for (int i = 0; i < pricePerDayList.length - 1; i++) {
                for (int j = i + 1; j < pricePerDayList.length; j++) {
                    int profit = sum(pricePerDayList, i, j);

                    if (profit > maxProfit) {
                        maxProfit = profit;
                        startDate = i + 1;
                        endDate = j + 1;
                    }
                }
            }
            int[] returnArray = { maxProfit, startDate, endDate };
            return returnArray;
        }

        // Returns the sum of the elements in the array from start to end
        private static int sum(int[] arr, int start, int end) {
            int sum = 0;
            for (int i = start + 1; i <= end; i++) {
                sum += arr[i];
            }
            return sum;
        }
    }

    // Class for generating tables with random values between -10 and 10
    public static class Table {

        int[] table;
        Random random = new Random();

        public Table(int[] table) {
            this.table = table;
        }

        public Table generateTable(int columns) {
            int[] newTable = new int[columns];

            for (int i = 0; i < columns; i++) {
                int randomInt = random.nextInt(21) - 10;
                newTable[i] = randomInt;
                int current = i + 1;
            }
            return new Table(newTable);
        }

    }
}
