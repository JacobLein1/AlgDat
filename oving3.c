#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
/*.
 *  This file contains the solution to the third exercise
 *  The task involves utilising quicSort() algorithm for sorting.
 *  https://www.idi.ntnu.no/emner/idatt2101/sortering/sortering-opg03.pdf
 *
 *
*/
// --------------------------------------------- //
//                    Utility                    //
// --------------------------------------------- //
/**
 * Swaps two integers
 *
 * @param a Pointer to the first integer
 * @param b Pointer to the second integer
*/
void swap(int* a, int* b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

/**
 * Sums an array
 *
 * @param arr Array to be summed
 * @param n Length of the array
*/
int sum_array(int arr[], int n) {
    int sum = 0;
    if (n == 0) {
        return 0;
    } else {
        for (int i = 0; i < n; i++)
        {
            sum += arr[i];
        }
    }
    return sum;
}

/**
 * Checks if an int array is sorted in increasing order
 *
 * @param arr Array to be checked
 * @param n Length of the array
*/
bool is_sorted(int arr[], int n) {
	if (n == 0) {
		return true;
	} else {
		for (int i = 0; i < n-1; i++)
		{
			if (arr[i] > arr[i+1]) {
				return false;
			}
		}
	}
	return true;
}

/**
 * Presents an int array with info like its sum and weather if it is sorted
 *
 * @param arr Array to be printed
 * @param n Length of the array
*/
void present_array(int arr[], int n) {
    for (int i = 0; i < n; i++)
        printf("%d ", arr[i]);
    printf("\n");
    printf("Sum: %d\n", sum_array(arr, n));
    printf("Sorted: %s\n", is_sorted(arr, n) ? "true" : "false");
}

// --------------------------------------------- //
//                  Generators                   //
// --------------------------------------------- //
int* generate_array(int n) {
    int* arr = (int*)malloc(n * sizeof(int));
    if (arr == NULL) {
        printf("Memory allocation failed!\n");
        exit(1);
    }
    for (int i = 0; i < n; i++) {
        arr[i] = rand() % 100;
    }
    return arr;
}

int* generate_array_duplicates(int n) {
    int* arr = (int*)malloc(n * sizeof(int));
    if (arr == NULL) {
        printf("Memory allocation failed!\n");
        exit(1);
    }
    for (int i = 0; i < n; i++) {
        if (i % 2 == 0) {
            arr[i] = 7;
        } else {
            arr[i] = rand() % 100;
        }
    }
    return arr;
}

int* generate_array_sorted(int n) {
    int* arr = (int*)malloc(n * sizeof(int));
    if (arr == NULL) {
        printf("Memory allocation failed!\n");
        exit(1);
    }
    for (int i = 0; i < n; i++) {
        arr[i] = i;
    }
    return arr;
}

int* generate_array_sorted_reversed(int n) {
    int* arr = (int*)malloc(n * sizeof(int));
    if (arr == NULL) {
        printf("Memory allocation failed!\n");
        exit(1);
    }
    for (int i = 0; i < n; i++) {
        arr[i] = n - i;
    }
    return arr;
}

// --------------------------------------------- //
//                   QuickSort                   //
// --------------------------------------------- //
int medianThreeSort(int arr[], int left, int right) {
    int mid = (left + right) / 2;

    if (arr[left] > arr[mid]) {
        swap(&arr[left], &arr[mid]);
    }
    if (arr[mid] > arr[right]) {
        swap(&arr[mid], &arr[right]);
    }
    if (arr[left] > arr[right]) {
        swap(&arr[left], &arr[right]);
    }

    return mid;
}

int normal_partition(int arr[], int low, int high) {

    // Choose the pivot element with median of three
    int pivotIndex = medianThreeSort(arr, low, high);
    int pivot = arr[pivotIndex];
    swap(&arr[pivotIndex], &arr[high]); // Move pivot to start

    int i = low;
    int j = high - 1;

    while (true) {

        // Find the first element greater than
        // the pivot (from starting)
        while (arr[i] < pivot && i < high) {
            i++;
        }

        // Find the first element smaller than
        // the pivot (from last)
        while (arr[j] > pivot && j >= low) {
            j--;
        }

        // if the pointers cross each other break
        if (i >= j) {
            break;
        }

        // Swap the elements
        swap(&arr[i], &arr[j]);

        // Moove the pointers
        i++;
        j--;
    }

    swap(&arr[high], &arr[i]);
    return i;
}

void quickSort(int arr[], int low, int high) {
    if (low < high) {

        // call partition function to find Partition Index
        int pi = normal_partition(arr, low, high);

        // Recursively call quickSort() for left and right
        // half based on Partition Index
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}

// --------------------------------------------- //
//             Dual Pivot QuickSort              //
// --------------------------------------------- //
void dual_partition(int arr[], int low, int high, int* lp, int* rp) {
    if (arr[low] > arr[high]) {
        swap(&arr[low], &arr[high]);
    }

    // Initialize variables
    int j = low + 1, g = high - 1, k = low + 1, p = arr[low], q = arr[high];

    while (k <= g) {

        // If elements are less than the smaller pivot
        if (arr[k] < p) {
            swap(&arr[k], &arr[j]);
            j++;
        }

        // If elements are greater than or equal to the larger pivot
        else if (arr[k] >= q) {
            while (arr[g] > q && k < g) {
                g--;
            }
            swap(&arr[k], &arr[g]);
            g--;

            if (arr[k] < p) {
                swap(&arr[k], &arr[j]);
                j++;
            }
        }
        k++;
    }
    j--;
    g++;

    // Swap pivots to their correct position
    swap(&arr[low], &arr[j]);
    swap(&arr[high], &arr[g]);

    // Update left and right pivot pointers
    *lp = j;
    *rp = g;
}

void dual_pivot_quickSort(int arr[], int low, int high) {
    if (low < high) {

        // lp means left pivot, and rp means right pivot
        int lp, rp;

        // shit bytte her
        swap(&arr[low], &arr[low+(high-low)/3]);
        swap(&arr[high], &arr[high-(high-low)/3]);

        // lp and rp will be updated by the partition function
        dual_partition(arr, low, high, &lp, &rp);

        // Recursively sort the subarrays unless all elements are equal
        dual_pivot_quickSort(arr, low, lp - 1);
        if (arr[lp] != arr[rp]) { dual_pivot_quickSort(arr, lp + 1, rp - 1); }
        dual_pivot_quickSort(arr, rp + 1, high);
    }
}

// --------------------------------------------- //
//                     Main                      //
// --------------------------------------------- //
int main() {
    printf("\n");

    for (int i = 0; i < 4; i++)
    {
        clock_t start = 0, end = 0;
         int n = 1000;
         int* arr;

         while (end - start < 1000000)
         {
             n *= 10;
             arr = generate_array(n);

             start = clock();
             quickSort(arr, 0, n-1);
             end = clock();

             long time = (end - start) * 1000 / CLOCKS_PER_SEC;
             printf("n = %15d :  %15ld ms\n", n, time);

             free(arr);
         }

         printf("\n");
     }

    return 0;
}