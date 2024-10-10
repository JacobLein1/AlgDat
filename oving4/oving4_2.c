
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
long kol;
// ----------------------------------- //
//                                     //
//                                     //
//           Hash table                //
//                                     //
//                                     //
// ----------------------------------- //
typedef struct HashTable {

long *table;

int size;
} HashTable;
// Oppretter en ny hashtabell
HashTable* create_table(int size) {

HashTable *new_table = malloc(sizeof(HashTable));

new_table->table = malloc(size * sizeof(long));

new_table->size = size;

for (int i = 0; i < size; i++) {

new_table->table[i] = 0;

}

return new_table;
}
// Primær hashfunksjon
unsigned int hash1(HashTable *table, long key) {

return key % table->size;
}
// Sekundær hashfunksjon
unsigned int hash2(HashTable *table, long key) {

return 1 + (key % (table->size - 1));
}
// Leter etter en ledig plass i hashtabellen med dobbel hashing
int probe(HashTable *table, long key) {

// Første forsøk

int pos = hash1(table, key), value;

// printf("%5ld h1 at %5d\n", key, pos);

// Finn verdien ved posisjonen

value = table->table[pos];

// Hvis plassen er ledig eller inneholder samme verdi

if (value == 0 || value == key) {

return pos;

}

// Ellers

int h2 = hash2(table, key);

while (true) {

kol++;

pos = (pos + h2) % table->size;

// printf("%5ld h2 at %5d\n", key, pos);

// Finn verdien ved posisjonen

value = table->table[pos];

// Hvis plassen er ledig eller inneholder samme verdi

if (value == 0 || value == key) {

//printf("Collisios %d\n", i);

return pos;

}

};
}
// Setter inn et tall i hashtabellen
void insert(HashTable *table, long key) {

int pos = probe(table, key);

table->table[pos] = key;

//printf("%5ld put at %5d\n", key, pos);
}
// Søker etter et tall i hashtabellen
int search(HashTable *table, long key) {

int pos = probe(table, key);

if (table->table[pos] == key) {

return pos;

}

// Hvis ikke funnet

return -1;
}
// Fjerner et tall fra hashtabellen

void delete(HashTable *table, long key) {

int pos = search(table, key);

if (pos != -1) {

table->table[pos] = 0;

}
}
// Frigjør minnet brukt av hashtabellen
void free_table(HashTable *table) {

free(table->table);

free(table);
}
// ----------------------------------- //
//                                     //
//                                     //
//            Test program             //
//                                     //
//                                     //
// ----------------------------------- //
// Genererer et array med tilfeldige tall i et gitt intervall
long* generate_array(int size, long range) {

// printf("Generating array of size %10d with numbers in range [-%ld, %ld]\n", size, range, range);

long *array = malloc(size * sizeof(long));

for (int i = 0; i < size; i++) {

array[i] = ((unsigned long)rand() * rand()) % range; // rand() lager ikke store nok tall så vi ganger to sammen

}

return array;
}
// Testprogram
int main()
{

printf("Testprogram for hashtabell\n");

printf("%10s %10s %10s %15s %15s %20s\n", "n", "m", "time", "collisions", "load factor", "collisions/insert");

kol = 0;

int n = 10000000;

int m = 13499999;

long* arr = generate_array(n, (long)n * 100L);

HashTable *table = create_table(m);

clock_t start = clock();

for (int i = 0; i < n; i++) {

insert(table, arr[i]);

}

clock_t end = clock();

double time = (double)(end - start) / CLOCKS_PER_SEC;

// Leter et tall

for (int i = 0; i < n; i++) {

if (search(table, arr[i]) == -1) {

printf("Error: %ld not found\n", arr[i]);

}

}

// Regner lastfaktor

int count = 0;

for (int i = 0; i < m; i++) {

if (table->table[i] != 0) {

count++;

}

}

float load_factor = (float)count / m;

// Regn kollisjoner per innsettelse

float collisions_per_insert = (float)kol / n;

printf("%10d %10d %10f %15ld %15f %20f\n",

n,

m,

time,

kol,

load_factor,

collisions_per_insert

);

free_table(table);
    free(arr);

return 0;
}